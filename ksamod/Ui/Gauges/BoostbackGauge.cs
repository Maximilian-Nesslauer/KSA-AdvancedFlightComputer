using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Navbox.Flight;

// The Boostback tab's content. The gauge shell, the tab bar and the EXECUTE/ABORT
// buttons live in Ui/Panel.cs; everything here draws inside the body child that panel
// opens, so it is plain ImGui under ConsoleStyle's widget styling.
//
// WHAT THIS TAB IS, FOR NOW. An aero workbench, not a guidance mode. It samples the
// focused vehicle's drag off KSA's own aerodynamics, fits the Cd(Mach, alpha)
// surrogate the SCvx formulation wants, and mirrors the game's atmosphere into a
// self-contained rho(h) the solver can carry without referencing the game. There is
// no boostback guidance behind EXECUTE yet - the buttons stripe out on this tab - and
// nothing here feeds the 6-DOF dynamics, which still have no aero term at all.
//
// The point of doing it as a visible tab rather than a silent setup step is that the
// surrogate is the thing most likely to be quietly wrong. A table sampled off the
// wrong frame, referenced to the wrong area, or built from a stale bounding box all
// produce a plausible-looking spline; the only way to catch it is to be able to read
// the numbers against a vehicle you can see.
public static partial class PoweredGuidanceWindow
{
    /// <summary>Alphas shown in the profile readout, degrees, retrograde-first. A
    /// selection rather than every breakpoint - twenty rows is a scroll, nine is a
    /// shape.</summary>
    private static readonly double[] BoostbackProfileAlphas =
        { 0.0, 5.0, 15.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0 };

    private static void DrawBoostbackTabContent(Vehicle vehicle, Orbit orbit,
                                                IParentBody parent, float innerW)
    {
        // Sample on first sight of a vehicle, and again whenever its bounding box
        // changes - which is the only thing that can change the answer, since KSA
        // itself only recomputes AerodynamicCdABody on a part-tree modification.
        //
        // "When the tab is selected" is exactly here: this method only runs while the
        // tab is open, so an unopened tab costs nothing and an open one re-fits only
        // when staging has actually invalidated the table.
        double3 live = LiveBoxExtents(vehicle);
        if (_s.AeroStale(live))
            ResampleBoostbackAero(vehicle, parent);

        DrawBoostbackSurrogateSection(vehicle, parent, innerW);
        DrawBoostbackProfileSection(innerW);
        DrawBoostbackAtmosphereSection(vehicle, orbit, parent, innerW);
    }

    /// <summary>The live bounding box, in the same terms the sweep records it.</summary>
    private static double3 LiveBoxExtents(Vehicle vehicle)
    {
        if (vehicle == null)
            return default;
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        return new double3(half.X * 2.0, half.Y * 2.0, half.Z * 2.0);
    }

    private static void ResampleBoostbackAero(Vehicle vehicle, IParentBody parent)
    {
        if (KsaAeroSweep.TryBuild(vehicle, parent, SimNow(),
                                  out KsaAeroSweep.Result result, out string error))
        {
            _s.Aero = result;
            _s.AeroError = "";
        }
        else
        {
            // Keep whatever we had. A failed resample on a vehicle mid-staging is not
            // a reason to throw away a table that was correct a second ago, and the
            // stale check will try again on the next frame anyway.
            _s.AeroError = error;
        }
    }

    // --- Surrogate ----------------------------------------------------------
    // What was sampled, and the three numbers that say whether it is worth trusting.
    private static void DrawBoostbackSurrogateSection(Vehicle vehicle, IParentBody parent,
                                                     float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Aero surrogate",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        KsaAeroSweep.Result a = _s.Aero;

        if (a == null)
        {
            GaugeRowText("Surrogate", _s.AeroError.Length > 0 ? _s.AeroError : "not sampled",
                new float4(1f, 0.4f, 0.4f, 1f));
        }
        else
        {
            GaugeRowText("Grid", $"{a.MachCount} Mach x {a.AlphaCount} alpha");
            GaugeRowText("Ref area", $"{a.ReferenceArea,8:F2} m^2  (nose face)");
            GaugeRowText("Skin area", $"{a.SkinArea,8:F1} m^2  (box surface)");
            GaugeRowText("Box (x,y,z)",
                $"{a.BoxExtents.X,6:F1} {a.BoxExtents.Y,5:F1} {a.BoxExtents.Z,5:F1} m");

            GaugeRowText("", "");

            // The three headline attitudes. Tail-first is the one boostback flies.
            GaugeRowText("Cd tail-first", $"{a.CdTailFirst,8:F2}   (alpha 0)");
            GaugeRowText("Cd broadside", $"{a.CdBroadside,8:F2}   (alpha 90)");
            GaugeRowText("Cd nose-first", $"{a.CdNoseFirst,8:F2}   (alpha 180)");

            float4 warn = new float4(1f, 0.8f, 0.3f, 1f);
            float4 dim = new float4(0.7f, 0.7f, 0.7f, 1f);

            // Two different questions about attitude, and they have opposite answers
            // for a slender booster - which is why both are here.
            //
            // FORM FRACTION is local, at alpha = 0. KSA adds 0.1 * (box surface area)
            // to CdA isotropically, and in the tail-first attitude that term swamps
            // the form drag, so a few degrees of pointing error near the boostback
            // attitude costs almost nothing. That is load-bearing for guidance.
            bool formMatters = a.FormFraction > 0.15;
            GaugeRowText("Form frac (a=0)", $"{a.FormFraction * 100.0,7:F1} %",
                formMatters ? dim : warn);
            if (!formMatters)
            {
                ImGui.Text("");
                ImGui.NextColumn();
                ImGui.TextWrapped("Near alpha 0 the drag is mostly KSA's isotropic "
                                + "skin term, so small pointing errors cost little.");
                ImGui.NextColumn();
            }

            // ATTITUDE SENSITIVITY is global. Broadside form drag is enormous whatever
            // the fraction above says, because a slender stack's flank area dwarfs its
            // nose area - so the alpha axis is carrying real information even when the
            // vehicle is insensitive to attitude where it normally sits.
            GaugeRowText("Cd(90)/Cd(0)", $"{a.AttitudeSensitivity,7:F1} x", dim);

            // Roll dependence the table cannot represent, because it has no roll input
            // and stores the azimuthal mean. This does NOT go to zero for an
            // axisymmetric vehicle: KSA's model is a box, so a square-section booster
            // rolled 45 degrees still presents sqrt(2) the area it does at 0. About
            // 25% is the floor for a slender stack; above ~35% the cross-section is
            // genuinely not square on top of that.
            GaugeRowText("Roll spread", $"{a.RollSpread * 100.0,7:F1} %  (~25% is inherent)",
                a.RollSpread > 0.35 ? warn : dim);

            GaugeRowText("Mach axis", "flat - KSA models no compressibility", warn);

            if (_s.AeroError.Length > 0)
                GaugeRowText("Last resample", _s.AeroError, new float4(1f, 0.4f, 0.4f, 1f));
        }

        ImGui.Text("");
        ImGui.NextColumn();
        if (ImGui.Button("RESAMPLE"))
            ResampleBoostbackAero(vehicle, parent);
        if (a != null)
        {
            ImGui.SameLine();
            if (ImGui.Button("COPY CSV"))
                ImGui.SetClipboardText(KsaAeroSweep.ToCsv(a));
        }
        ImGui.NextColumn();

        ImGuiHelper.EndRegion();
    }

    // --- Cd profile ---------------------------------------------------------
    // The table itself, read off the FITTED spline rather than the sampled grid, so
    // what is shown is what the solver would actually get - including any overshoot
    // the fit introduced between breakpoints.
    private static void DrawBoostbackProfileSection(float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Cd profile", ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        KsaAeroSweep.Result a = _s.Aero;
        if (a?.Table == null)
        {
            GaugeRowText("Profile", "no table");
            ImGuiHelper.EndRegion();
            return;
        }

        // One Mach is enough while the axis is flat; sampling at 0.8 rather than 0
        // means a future non-flat table shows something representative here without
        // this needing to change.
        const double AtMach = 0.8;
        const double Deg = Math.PI / 180.0;

        for (int i = 0; i < BoostbackProfileAlphas.Length; i++)
        {
            double deg = BoostbackProfileAlphas[i];
            double cd = a.Table.Cd(AtMach, deg * Deg);
            // CdA is the number that actually multiplies dynamic pressure, so show it
            // beside the coefficient - it is what makes the drag force checkable
            // against the game without doing arithmetic in your head.
            GaugeRowText($"alpha {deg,5:F0} deg",
                $"Cd {cd,7:F2}   CdA {cd * a.ReferenceArea,8:F1} m^2");
        }

        ImGui.Text("");
        ImGui.NextColumn();
        ImGui.TextWrapped("Alpha is RETROGRADE-FIRST: 0 is engine into the wind, "
                        + "180 is nose-first.");
        ImGui.NextColumn();

        ImGuiHelper.EndRegion();
    }

    // --- Atmosphere ---------------------------------------------------------
    // The mirrored rho(h), and the check that it really is a mirror.
    private static void DrawBoostbackAtmosphereSection(Vehicle vehicle, Orbit orbit,
                                                       IParentBody parent, float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Atmosphere",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        ExponentialAtmosphere atm = _s.Aero?.Atmosphere;
        if (atm == null)
        {
            GaugeRowText("Atmosphere", "none on this body");
            ImGuiHelper.EndRegion();
            return;
        }

        GaugeRowText("Body", _s.Aero.BodyName.Length > 0 ? _s.Aero.BodyName : "(unnamed)");
        GaugeRowText("Sea level rho", $"{atm.SeaLevelDensity,8:F4} kg/m^3");
        GaugeRowText("Sea level P", $"{atm.SeaLevelPressure / 1000.0,8:F2} kPa");
        GaugeRowText("Scale height", $"{atm.ScaleHeight / 1000.0,8:F2} km");
        GaugeRowText("Top", $"{atm.TopAltitude / 1000.0,8:F1} km");
        // Derived from P0/rho0, not assumed - an isothermal atmosphere has one speed
        // of sound at every altitude. See ExponentialAtmosphere.
        GaugeRowText("Speed of sound", $"{atm.SpeedOfSound,8:F1} m/s  (derived)");

        // The mirror check. This is the number that says our self-contained rho really
        // is the game's rho, re-verified on every resample rather than assumed once.
        double err = _s.Aero.AtmosphereMirrorError;
        GaugeRowText("Mirror error", $"{err:E2}",
            err < 1e-9 ? new float4(0.4f, 1f, 0.4f, 1f) : new float4(1f, 0.4f, 0.4f, 1f));

        GaugeRowText("", "");

        // --- live, so the model can be checked against a vehicle in flight ---
        double alt = orbit.StateVectors.PositionCci.Length() - parent.MeanRadius;
        double rho = atm.Density(alt);
        double speed = vehicle.GetSurfaceSpeed();
        double q = 0.5 * rho * speed * speed;

        GaugeRowText("Altitude", $"{alt / 1000.0,8:F2} km");
        GaugeRowText("rho", $"{rho,8:E3} kg/m^3");
        GaugeRowText("Airspeed", $"{speed,8:F1} m/s");
        GaugeRowText("Mach", $"{atm.Mach(speed),8:F2}");
        GaugeRowText("q", $"{q / 1000.0,8:F2} kPa");

        if (TryLiveAlpha(vehicle, orbit, parent, out double alphaDeg))
        {
            GaugeRowText("Alpha (live)", $"{alphaDeg,8:F1} deg");
            KsaAeroSweep.Result a = _s.Aero;
            if (a?.Table != null)
            {
                double cd = a.Table.Cd(atm.Mach(speed), alphaDeg * Math.PI / 180.0);
                // The whole chain, end to end: what the surrogate says this vehicle is
                // feeling right now. Comparable against the game by eye.
                GaugeRowText("Drag (model)", $"{cd * a.ReferenceArea * q / 1000.0,8:F1} kN");
            }
        }
        else
        {
            GaugeRowText("Alpha (live)", "stationary");
        }

        ImGuiHelper.EndRegion();
    }

    /// <summary>
    /// The vehicle's CURRENT angle of attack, retrograde-first, in degrees.
    ///
    /// Airspeed is surface-relative because KSA's atmosphere co-rotates rigidly with
    /// the body - the game subtracts omega x r itself before computing drag, and a
    /// readout that used inertial velocity would disagree with the sim by a full
    /// equatorial rotation speed near the ground.
    ///
    /// Measured in KSA's OWN body frame, where +x is the nose, rather than the
    /// solver's model frame where the thrust axis is +z. That is deliberate: the
    /// surrogate was sampled against AerodynamicCdABody, which is indexed in these
    /// axes, so reading it back the same way is what makes the two comparable.
    /// </summary>
    private static bool TryLiveAlpha(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                     out double alphaDeg)
    {
        alphaDeg = 0.0;

        double3 r = orbit.StateVectors.PositionCci;
        double3 vAir = orbit.StateVectors.VelocityCci
                     - double3.Cross(parent.GetAngularVelocityCci(), r);
        if (vAir.LengthSquared() < 1e-6)
            return false;

        double3 vBody = vAir.Transform(doubleQuat.Inverse(vehicle.GetBody2Cci()));

        // atan2 of cross-flow against the TAIL-ward axial component: -x, so a vehicle
        // flying engine-first reads zero.
        double cross = Math.Sqrt(vBody.Y * vBody.Y + vBody.Z * vBody.Z);
        alphaDeg = Math.Atan2(cross, -vBody.X) * 180.0 / Math.PI;
        return true;
    }
}
