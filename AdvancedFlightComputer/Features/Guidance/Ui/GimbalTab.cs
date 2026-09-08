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
            if (_s.GimbalMode != 0)
            {
                _s.GimbalMode = 0;
                KsaGimbalControl.Disengage(vehicle);
            }
            return;
        }

        int previousMode = _s.GimbalMode;
        ImGui.RadioButton("Off", ref _s.GimbalMode, 0);
        ImGui.SameLine();
        ImGui.RadioButton("Direct", ref _s.GimbalMode, 1);
        ImGui.SameLine();
        ImGui.RadioButton("KSA torque", ref _s.GimbalMode, 2);
        ImGui.SameLine();
        ImGui.RadioButton("Torque (N-m)", ref _s.GimbalMode, 3);

        if (_s.GimbalMode != previousMode && _s.GimbalMode == 0)
            KsaGimbalControl.Disengage(vehicle);

        if (_s.GimbalMode == 0)
        {
            ImGui.TextWrapped("The flight computer has normal control of these engines.");
            DrawGimbalTable(vehicle, gimbals);
            return;
        }

        // No re-pointing any more: the override is keyed on the vehicle's config, so
        // switching craft or staging simply lands on a different slot rather than
        // dragging one global target around behind the player.

        ImGui.TextWrapped(
            "While engaged the flight computer's own attitude control cannot move these " +
            "engines. Switch back to Off to hand them back.");

        if (_s.GimbalMode == 1)
        {
            ImGui.SliderFloat("Gimbal Y", ref _s.GimbalY, -1f, 1f);
            ImGui.SliderFloat("Gimbal Z", ref _s.GimbalZ, -1f, 1f);
            if (ImGui.Button("Zero"))
                _s.GimbalY = _s.GimbalZ = 0f;

            KsaGimbalControl.SetDirect(vehicle, _s.GimbalY, _s.GimbalZ);
        }
        else if (_s.GimbalMode == 2)
        {
            ImGui.TextWrapped(
                "KSA's own heuristic: a dimensionless demand offered to every gimbal. " +
                "Produces roll, but the magnitude is not calibrated and the result is " +
                "not parallel to the demand.");
            ImGui.SliderFloat("Roll (body X)", ref _s.GimbalRoll, -1f, 1f);
            ImGui.SliderFloat("Pitch (body Y)", ref _s.GimbalPitch, -1f, 1f);
            ImGui.SliderFloat("Yaw (body Z)", ref _s.GimbalYaw, -1f, 1f);
            if (ImGui.Button("Zero"))
                _s.GimbalRoll = _s.GimbalPitch = _s.GimbalYaw = 0f;

            KsaGimbalControl.SetTorque(vehicle, _s.GimbalRoll, _s.GimbalPitch, _s.GimbalYaw);
        }
        else
        {
            DrawLsqControls(vehicle);
        }

        int applied = KsaGimbalControl.Diagnostics(vehicle)?.AppliedCount ?? 0;
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
    private static void DrawLsqControls(Vehicle vehicle)
    {
        ImGui.TextWrapped(
            "Least-squares allocation: solves for the deflections that deliver the " +
            "commanded torque. This is the interface a guidance mode should use.");

        TvcAllocationResult a = KsaGimbalControl.Diagnostics(vehicle)?.LastAllocation ?? default;
        double maxRoll = Math.Abs(a.MaxTorque.X);
        double maxPitch = Math.Abs(a.MaxTorque.Y);
        double maxYaw = Math.Abs(a.MaxTorque.Z);

        ImGui.SliderFloat("Roll (body X)", ref _s.GimbalRoll, -1f, 1f);
        ImGui.SliderFloat("Pitch (body Y)", ref _s.GimbalPitch, -1f, 1f);
        ImGui.SliderFloat("Yaw (body Z)", ref _s.GimbalYaw, -1f, 1f);
        if (ImGui.Button("Zero"))
            _s.GimbalRoll = _s.GimbalPitch = _s.GimbalYaw = 0f;

        var lsq = new double3(_s.GimbalRoll * maxRoll, _s.GimbalPitch * maxPitch, _s.GimbalYaw * maxYaw);
        KsaGimbalControl.SetLsq(vehicle, lsq);

        ImGui.Separator();
        ImGui.Text($"Capability  R {maxRoll / 1000.0,9:F1}  P {maxPitch / 1000.0,9:F1}  Y {maxYaw / 1000.0,9:F1}  kN-m");
        ImGui.Text($"Commanded   R {lsq.X / 1000.0,9:F1}  " +
                   $"P {lsq.Y / 1000.0,9:F1}  " +
                   $"Y {lsq.Z / 1000.0,9:F1}  kN-m");
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
                $"Saturated - demand scaled to {a.SaturationScale * 100.0:F0}%% (direction preserved).");
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
        var demand = new double3(_s.GimbalRoll, _s.GimbalPitch, _s.GimbalYaw);

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
            if (_s.GimbalMode == 1)
            {
                cmdY = _s.GimbalY;
                cmdZ = _s.GimbalZ;
            }
            else if (_s.GimbalMode == 3)
            {
                // The Lsq solve is global, not per-gimbal, so there is nothing to
                // recompute here — show what the worker actually commanded.
                KsaGimbalControl.Slot lsqSlot = KsaGimbalControl.Diagnostics(vehicle);
                ReadOnlySpan<double> last = lsqSlot != null ? lsqSlot.LastCommands : default;
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
