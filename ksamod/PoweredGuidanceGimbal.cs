using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// "Gimbal" tab — a manual probe for KsaGimbalControl.
//
// Not a guidance mode. Its job is to answer the question the 6-DOF port is blocked
// on: can the mod command thrust vectoring directly, and at what layer? Two modes:
//
//   Direct  - same normalized deflection to every gimbal. Proves the write lands.
//   Torque  - body-frame roll/pitch/yaw through KSA's own per-gimbal geometry.
//             This is the layer a guidance mode should actually use.
//
// The per-gimbal table shows which body axes each gimbal has leverage over, which
// is how a main engine and its roll verniers visibly separate.
public static partial class PoweredGuidanceWindow
{
    private static int _gimbalMode;          // 0 = off, 1 = direct, 2 = torque
    private static float _gimbalY;
    private static float _gimbalZ;
    private static float _gimbalRoll;
    private static float _gimbalPitch;
    private static float _gimbalYaw;

    private static void DrawGimbalTab(Vehicle vehicle)
    {
        ImGui.TextWrapped(
            "Writes thrust vectoring straight into the flight computer's TVC output, " +
            "replacing its own attitude allocation. Test tool, not a guidance mode.");
        ImGui.Separator();

        Span<GimbalController> gimbals = vehicle.Parts.Modules.Get<GimbalController>();

        if (gimbals.Length == 0)
        {
            ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f),
                "This vehicle has no gimballed engines - nothing to command.");
            if (_gimbalMode != 0)
            {
                _gimbalMode = 0;
                KsaGimbalControl.Disengage();
            }
            return;
        }

        int previousMode = _gimbalMode;
        ImGui.RadioButton("Off", ref _gimbalMode, 0);
        ImGui.SameLine();
        ImGui.RadioButton("Direct", ref _gimbalMode, 1);
        ImGui.SameLine();
        ImGui.RadioButton("KSA torque", ref _gimbalMode, 2);
        ImGui.SameLine();
        ImGui.RadioButton("Torque (N-m)", ref _gimbalMode, 3);

        if (_gimbalMode != previousMode && _gimbalMode == 0)
            KsaGimbalControl.Disengage();

        if (_gimbalMode == 0)
        {
            ImGui.TextWrapped("The flight computer has normal control of these engines.");
            DrawGimbalTable(vehicle, gimbals);
            return;
        }

        // Re-point at the live config every frame: the player can switch vehicles,
        // and staging rebuilds the part tree underneath us.
        KsaGimbalControl.SetTarget(vehicle);

        ImGui.TextWrapped(
            "While engaged the flight computer's own attitude control cannot move these " +
            "engines. Switch back to Off to hand them back.");

        if (_gimbalMode == 1)
        {
            ImGui.SliderFloat("Gimbal Y", ref _gimbalY, -1f, 1f);
            ImGui.SliderFloat("Gimbal Z", ref _gimbalZ, -1f, 1f);
            if (ImGui.Button("Zero"))
                _gimbalY = _gimbalZ = 0f;

            KsaGimbalControl.CommandY = _gimbalY;
            KsaGimbalControl.CommandZ = _gimbalZ;
            KsaGimbalControl.Mode = GimbalOverrideMode.Direct;
        }
        else if (_gimbalMode == 2)
        {
            ImGui.TextWrapped(
                "KSA's own heuristic: a dimensionless demand offered to every gimbal. " +
                "Produces roll, but the magnitude is not calibrated and the result is " +
                "not parallel to the demand.");
            ImGui.SliderFloat("Roll (body X)", ref _gimbalRoll, -1f, 1f);
            ImGui.SliderFloat("Pitch (body Y)", ref _gimbalPitch, -1f, 1f);
            ImGui.SliderFloat("Yaw (body Z)", ref _gimbalYaw, -1f, 1f);
            if (ImGui.Button("Zero"))
                _gimbalRoll = _gimbalPitch = _gimbalYaw = 0f;

            KsaGimbalControl.TorqueRoll = _gimbalRoll;
            KsaGimbalControl.TorquePitch = _gimbalPitch;
            KsaGimbalControl.TorqueYaw = _gimbalYaw;
            KsaGimbalControl.Mode = GimbalOverrideMode.Torque;
        }
        else
        {
            DrawLsqControls();
        }

        int applied = KsaGimbalControl.AppliedCount;
        if (applied == 0)
            ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f),
                "Engaged, but reaching 0 gimbals - the override is not landing.");
        else
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                $"Commanding {applied} gimbal(s).");

        DrawGimbalTable(vehicle, gimbals);
    }

    // Physical torque command. Sliders are normalized for usability but scaled by the
    // allocator's own per-axis capability, so what you set is a real N-m demand — and
    // the readout below shows what the allocation actually delivers, including the
    // lateral force that necessarily comes with it.
    private static void DrawLsqControls()
    {
        ImGui.TextWrapped(
            "Least-squares allocation: solves for the deflections that deliver the " +
            "commanded torque. This is the interface a guidance mode should use.");

        TvcAllocationResult a = KsaGimbalControl.LastAllocation;
        double maxRoll = Math.Abs(a.MaxTorque.X);
        double maxPitch = Math.Abs(a.MaxTorque.Y);
        double maxYaw = Math.Abs(a.MaxTorque.Z);

        ImGui.SliderFloat("Roll (body X)", ref _gimbalRoll, -1f, 1f);
        ImGui.SliderFloat("Pitch (body Y)", ref _gimbalPitch, -1f, 1f);
        ImGui.SliderFloat("Yaw (body Z)", ref _gimbalYaw, -1f, 1f);
        if (ImGui.Button("Zero"))
            _gimbalRoll = _gimbalPitch = _gimbalYaw = 0f;

        KsaGimbalControl.TorqueXNm = _gimbalRoll * maxRoll;
        KsaGimbalControl.TorqueYNm = _gimbalPitch * maxPitch;
        KsaGimbalControl.TorqueZNm = _gimbalYaw * maxYaw;
        KsaGimbalControl.Mode = GimbalOverrideMode.Lsq;

        ImGui.Separator();
        ImGui.Text($"Capability  R {maxRoll / 1000.0,9:F1}  P {maxPitch / 1000.0,9:F1}  Y {maxYaw / 1000.0,9:F1}  kN-m");
        ImGui.Text($"Commanded   R {KsaGimbalControl.TorqueXNm / 1000.0,9:F1}  " +
                   $"P {KsaGimbalControl.TorqueYNm / 1000.0,9:F1}  " +
                   $"Y {KsaGimbalControl.TorqueZNm / 1000.0,9:F1}  kN-m");
        ImGui.Text($"Achieved    R {a.AchievedTorque.X / 1000.0,9:F1}  " +
                   $"P {a.AchievedTorque.Y / 1000.0,9:F1}  " +
                   $"Z {a.AchievedTorque.Z / 1000.0,9:F1}  kN-m");

        // Gimballing for torque always tilts the thrust vector. Surfacing it here
        // because the 6-DOF model has to account for it: it is a real acceleration,
        // not an artifact of the allocation.
        ImGui.Text($"Side force  X {a.AchievedForce.X / 1000.0,9:F1}  " +
                   $"Y {a.AchievedForce.Y / 1000.0,9:F1}  " +
                   $"Z {a.AchievedForce.Z / 1000.0,9:F1}  kN");

        if (a.SaturationScale < 1.0)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                $"Saturated - demand scaled to {a.SaturationScale * 100.0:F0}% (direction preserved).");
    }

    // Per-gimbal breakdown. The "axes" column is the interesting one: it shows which
    // body axes each gimbal can actually torque about, given its moment arm. A
    // centreline main engine reads "-PY" (pitch and yaw, no roll) while an off-axis
    // vernier reads "RPY" — which is how KSA gets roll authority without a dedicated
    // roll actuator.
    private static void DrawGimbalTable(Vehicle vehicle, Span<GimbalController> gimbals)
    {
        ImGui.Separator();

        FlightComputer fc = vehicle.FlightComputer;
        if (fc == null)
        {
            ImGui.Text($"Gimbals on vehicle: {gimbals.Length}");
            return;
        }

        float3 com = fc.CenterOfMassAsmb;
        var demand = new double3(_gimbalRoll, _gimbalPitch, _gimbalYaw);

        ImGui.Text($"Gimbals on vehicle: {gimbals.Length}   (axes = which body axes it can torque about)");

        for (int i = 0; i < gimbals.Length; i++)
        {
            GimbalController gc = gimbals[i];
            Gimbal g = gc.Gimbal;

            KsaGimbalControl.Leverage(gc, com, out bool roll, out bool pitch, out bool yaw);
            string axes = (roll ? "R" : "-") + (pitch ? "P" : "-") + (yaw ? "Y" : "-");

            float3 arm = gc.Data.ThrustPosVehicleAsmb - com;
            double maxY = g.AxisY.MaxAngle * 180.0 / Math.PI;
            double maxZ = g.AxisZ.MaxAngle * 180.0 / Math.PI;

            // Show the command this gimbal will actually receive, computed by the same
            // code the worker runs — so the table can't drift from the behaviour.
            float cmdY, cmdZ;
            if (_gimbalMode == 1)
            {
                cmdY = _gimbalY;
                cmdZ = _gimbalZ;
            }
            else if (_gimbalMode == 3)
            {
                // The Lsq solve is global, not per-gimbal, so there is nothing to
                // recompute here — show what the worker actually commanded.
                ReadOnlySpan<double> last = KsaGimbalControl.LastCommands;
                cmdY = last.Length > 2 * i + 1 ? (float)last[2 * i] : 0f;
                cmdZ = last.Length > 2 * i + 1 ? (float)last[2 * i + 1] : 0f;
            }
            else
            {
                KsaGimbalControl.Allocate(gc, com, demand, out cmdY, out cmdZ);
            }

            ImGui.Text(
                $"  #{i} [{axes}]  arm ({arm.X:F2},{arm.Y:F2},{arm.Z:F2}) m  " +
                $"Tmax {gc.Data.MaximumThrust / 1000.0:F0} kN  " +
                $"max {maxY:F1}/{maxZ:F1} deg  ->  {cmdY * maxY:F2}/{cmdZ * maxZ:F2} deg");
        }

        ImGui.Separator();
        ImGui.TextWrapped(
            "Deflection is commanded whether or not the engine is lit, so the nozzles " +
            "can be checked on the pad - but there is no torque without thrust.");
    }
}
