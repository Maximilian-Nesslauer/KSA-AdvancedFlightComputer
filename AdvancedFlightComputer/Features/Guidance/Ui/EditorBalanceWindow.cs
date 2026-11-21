using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using KSA.Aero;
using SysVector3 = System.Numerics.Vector3;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// BELLY-FLOP BALANCE, in the vehicle editor: where the centre of pressure sits against the centre of mass when the craft flies belly first, so it can be balanced before it flies.
///
/// The loads are the game's own: the stock local-inclination law (KSA.Aero.LocalInclinationLaw, the one PhysicsStates.ComputeAerodynamicLoads applies in flight) on the editor's own aero mesh of the craft (PartTree.Aerodynamics), at each angle of attack from nose-first with the belly (the craft's -Z, down on the navball at roll 0, or another side) into the air. For each angle: the moment about the centre of mass as filled in the editor, its coefficient on the scale the flight sweep uses (q S L, S the bounding box's frontal area, L its length), and the centre of pressure along the craft's long axis. The centre of mass has to move to the centre of pressure for the craft to trim there with its flaps at rest, and that distance is the row's answer. Positive offsets are towards the nose.
///
/// Flaps are at rest here: what they can add is in the flight log's "schedule with the flaps". Recomputed when the mesh or the mass changes, at most every half second.
/// </summary>
[HarmonyPatch(typeof(VehicleEditor), nameof(VehicleEditor.OnDrawUi), new[] { typeof(IGameViewport) })]
internal static class EditorBalanceWindow
{
    private const string WindowId = "afc-editor-balance";
    private static readonly double[] AnglesDeg = [30, 40, 50, 60, 70, 80, 90];
    private static readonly double[] MachRows = [5.0, 1.5];

    private static bool _open = true;
    private static VehicleEditor? _editor;
    private static int _side = 1;      // 0 -Z, 1 +Z (the craft's belly), 2 -Y, 3 +Y
    private static int _machRow;
    private static Aerodynamics? _computedFor;
    private static double _computedMass = double.NaN, _computedAt = double.NegativeInfinity;
    private static int _computedSide = -1, _computedMach = -1;
    private static readonly List<Row> _rows = new();
    private static double _lengthM, _noseX, _comX, _dryComX, _massKg, _dryMassKg;
    private static string _note = "";

    private readonly record struct Row(double AlphaDeg, double Cm, double CpFromComM, double CdS, double LiftToDrag);

    private static readonly SysVector3[] Sides = { new(0, 0, -1), new(0, 0, 1), new(0, -1, 0), new(0, 1, 0) };
    private static readonly string[] SideNames = { "-Z", "+Z (craft belly)", "-Y", "+Y" };

    static void Postfix(VehicleEditor __instance)
    {
        try
        {
            if (!ReferenceEquals(_editor, __instance))
            {
                _editor = __instance;
                _open = true;
                _computedFor = null;
            }
            if (_open)
                Draw(__instance);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-editor-balance:" + ex.GetType().Name, $"[AFC] Editor balance window failed: {ex}");
        }
    }

    private static void Draw(VehicleEditor editor)
    {
        PartTree? tree = editor.EditingSpace?.Parts;
        Recompute(tree);

        float s = ImGuiHelper.InterfaceScale;
        ConsoleStyle.EnsureInitialized();
        bool open = true;
        if (!ConsoleStyle.BeginWindow(WindowId.AsSpan(), "BELLY-FLOP BALANCE".AsSpan(), "AFC".AsSpan(), ref open,
                new float2(470f * s, 330f * s), ImGuiWindowFlags.NoScrollbar, ImGuiCond.FirstUseEver, pinToMainViewport: false))
            return;
        try
        {
            if (!open)
                _open = false;
            ConsoleLayout layout = ConsoleStyle.CurrentLayout;
            float pad = 8f * s;
            ImGui.SetCursorScreenPos(new float2(layout.BodyPos.X + pad, layout.BodyPos.Y + pad));
            ImGui.BeginGroup();
            if (_note.Length > 0)
                ImGui.Text(_note);
            else
            {
                ImGui.Text(FormattableString.Invariant($"Length {_lengthM:F1} m. CoM {_noseX - _comX:F2} m from the nose as filled ({_massKg / 1000:F1} t), {_noseX - _dryComX:F2} m dry ({_dryMassKg / 1000:F1} t)."));
                ImGui.Text("Belly side:");
                for (int i = 0; i < Sides.Length; i++)
                {
                    ImGui.SameLine();
                    if (ImGui.SmallButton((i == _side ? "[" + SideNames[i] + "]" : SideNames[i]) + "##afc-bal-side" + i))
                        _side = i;
                }
                ImGui.Text("Speed:");
                for (int i = 0; i < MachRows.Length; i++)
                {
                    ImGui.SameLine();
                    string label = FormattableString.Invariant($"Mach {MachRows[i]:0.#}");
                    if (ImGui.SmallButton((i == _machRow ? "[" + label + "]" : label) + "##afc-bal-mach" + i))
                        _machRow = i;
                }
                ImGui.Separator();
                ImGui.Text("AoA   C_m      CP from CoM     L/D   to trim, move the CoM");
                foreach (var r in _rows)
                {
                    string where = Math.Abs(r.CpFromComM) < 0.05 ? "trimmed"
                        : FormattableString.Invariant($"{Math.Abs(r.CpFromComM):F2} m towards the {(r.CpFromComM > 0 ? "nose" : "tail")}");
                    ImGui.Text(FormattableString.Invariant(
                        $"{r.AlphaDeg,3:F0}  {r.Cm,7:F3}   {(r.CpFromComM >= 0 ? "+" : "")}{r.CpFromComM,6:F2} m     {r.LiftToDrag,5:F2}   {where}"));
                }
                ImGui.Separator();
                ImGui.Text("AoA from nose-first. C_m + raises the AoA. CP + is towards the nose.");
                ImGui.Text("Flaps at rest; their reach is in the flight log's schedule with the flaps.");
            }
            ImGui.EndGroup();
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    private static void Recompute(PartTree? tree)
    {
        double now = Environment.TickCount64 / 1000.0;
        Aerodynamics? aero = tree?.Aerodynamics;
        if (tree == null || tree.IsDisposed)
        {
            _note = "No craft in the editor.";
            return;
        }
        if (aero == null || tree.IsRebuildingAerodynamics)
        {
            _note = "Waiting for the aero mesh.";
            return;
        }
        AeroSurfaceFlow? flow = aero.GetSurfaceFlow();
        if (flow == null)
        {
            _note = "The aero mesh has no surface flow.";
            return;
        }

        // The centre of mass as filled in the editor: the inert parts plus every store at its fill.
        OffsetMassProperties dry = tree.StaticMassPropsAsmb;
        OffsetMassProperties full = dry;
        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        foreach (ISubstanceStore store in tree.SubstanceStores)
            full += store.ComputeMassPropertiesVehicleAsmb(moles);
        double mass = full.Props.Mass;
        bool changed = !ReferenceEquals(aero, _computedFor) || Math.Abs(mass - _computedMass) > 1e-3 * Math.Max(mass, 1)
                       || _side != _computedSide || _machRow != _computedMach;
        // Only when something changed, and not more than twice a second while the craft is being edited.
        if (!changed || now - _computedAt < 0.5)
            return;
        _computedFor = aero;
        _computedMass = mass;
        _computedSide = _side;
        _computedMach = _machRow;
        _computedAt = now;
        _massKg = mass;
        _dryMassKg = dry.Props.Mass;
        _comX = full.Offset.X;
        _dryComX = dry.Offset.X;

        (SysVector3 min, SysVector3 max) = aero.WidthBounds;
        _lengthM = max.X - min.X;
        _noseX = max.X;
        double dy = max.Y - min.Y, dz = max.Z - min.Z;
        double refArea = Math.PI / 4.0 * dy * dz, refLength = _lengthM;
        if (!(refArea > 0) || !(refLength > 0))
        {
            _note = "The aero mesh has no size yet.";
            return;
        }

        IParentBody? air = AirBody();
        PhysicalAtmosphere? atmosphere = air?.GetPhysicalAtmosphere();
        if (atmosphere == null)
        {
            _note = "No body with an atmosphere to take the air from.";
            return;
        }
        double temperature = atmosphere.ComputeTemperatureAtAltitude(0.0);
        AtmosphericConditions sea = atmosphere.ComputeConditionsAtAltitude(0.0);
        double a = atmosphere.Mixture.ComputeSpeedOfSound(temperature);
        double mach = MachRows[_machRow];
        double speed = mach * a;
        var law = new LocalInclinationLaw((float)mach, (float)atmosphere.Mixture.ComputeGamma(temperature));
        var friction = new SkinFrictionLaw(atmosphere.Mixture, temperature, sea.Density, speed, null);

        SysVector3 b = Sides[_side];
        SysVector3 x = SysVector3.UnitX;
        SysVector3 pitch = SysVector3.Cross(b, x);
        SysVector3 com = new((float)full.Offset.X, (float)full.Offset.Y, (float)full.Offset.Z);
        _rows.Clear();
        foreach (double alphaDeg in AnglesDeg)
        {
            double alpha = alphaDeg * Math.PI / 180.0;
            // The craft moves along u = cos(a) x + sin(a) b, so the air flows along -u.
            SysVector3 u = (float)Math.Cos(alpha) * x + (float)Math.Sin(alpha) * b;
            SysVector3 flowDir = -u;
            flow.Update(flowDir);
            SurfaceLoads loads = law.ComputeLoads(flow, in friction, flowDir, SysVector3.Zero, com);
            SysVector3 f = loads.ForcePerDynamicPressure, m = loads.MomentPerDynamicPressure;
            double mPitch = SysVector3.Dot(m, pitch);
            double arm = SysVector3.Dot(SysVector3.Cross(x, f), pitch);
            double cpFromCom = Math.Abs(arm) > 1e-9 ? mPitch / arm : double.NaN;
            double dragS = -SysVector3.Dot(f, u);
            SysVector3 liftDir = (float)Math.Sin(alpha) * x - (float)Math.Cos(alpha) * b;
            double liftS = SysVector3.Dot(f, liftDir);
            _rows.Add(new Row(alphaDeg, mPitch / (refArea * refLength), cpFromCom, dragS, dragS > 1e-9 ? liftS / dragS : 0.0));
        }
        _note = "";
    }

    /// <summary>The body to take the air's gas from: Earth if the system has it, else the first body with an atmosphere.</summary>
    private static IParentBody? AirBody()
    {
        IParentBody? any = null;
        foreach (Celestial c in Universe.CurrentSystem.All.OfType<Celestial>())
        {
            if (c is not IParentBody body || body.GetPhysicalAtmosphere() == null)
                continue;
            if (c.Id == "Earth")
                return body;
            any ??= body;
        }
        return any;
    }
}
