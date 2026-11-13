using System;
using System.Collections.Generic;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// Reads and writes the guidance fields a <see cref="SettingSpec"/> names, so a page can count and
/// reset its changed values without a hand-written list of fields. A key that does not resolve logs
/// once and counts as unchanged, so a renamed field costs the count, not the page. A value that
/// guidance or the launch site writes by itself is not the player's change, so it is not counted,
/// not reset and not marked, see <see cref="SetBy"/>.
/// </summary>
internal static class SettingAccess
{
    private static readonly Dictionary<string, FieldInfo?> _fields = new(StringComparer.Ordinal);

    private static FieldInfo? Field(string key)
    {
        if (_fields.TryGetValue(key, out FieldInfo? cached))
            return cached;
        FieldInfo? field = typeof(VehicleAutopilotState).GetField(key, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        if (field != null && field.FieldType != typeof(double) && field.FieldType != typeof(int) && field.FieldType != typeof(bool))
            field = null;
        if (field == null)
            LogHelper.WarnOnce("setting-access:" + key, $"[AFC] Setting '{key}' has no double, int or bool field on VehicleAutopilotState.");
        _fields[key] = field;
        return field;
    }

    public static bool TryGetDouble(VehicleAutopilotState state, string key, out double value)
    {
        value = double.NaN;
        FieldInfo? field = Field(key);
        if (field == null)
            return false;
        object? raw = field.GetValue(state);
        switch (raw)
        {
            case double d: value = d; return true;
            case int i: value = i; return true;
            default: return false;
        }
    }

    public static void SetDouble(VehicleAutopilotState state, string key, double value)
    {
        FieldInfo? field = Field(key);
        if (field == null)
            return;
        if (field.FieldType == typeof(int))
            field.SetValue(state, (int)Math.Round(value));
        else if (field.FieldType == typeof(double))
            field.SetValue(state, value);
    }

    public static bool TryGetBool(VehicleAutopilotState state, string key, out bool value)
    {
        value = false;
        FieldInfo? field = Field(key);
        if (field == null || field.FieldType != typeof(bool))
            return false;
        value = (bool)field.GetValue(state)!;
        return true;
    }

    public static void SetBool(VehicleAutopilotState state, string key, bool value)
    {
        FieldInfo? field = Field(key);
        if (field != null && field.FieldType == typeof(bool))
            field.SetValue(state, value);
    }

    /// <summary>
    /// Who writes the value of <paramref name="key"/> now, when it is not the player: the chase of a picked target that is found on this body (ApplyChaseOrbit), the launch site that seeds the LAN, the craft whose real minimum throttle sets the touchdown guidance floor (TryConfigure6Dof), or the colliders of the craft that set the vehicle height. Empty when the player owns the value.
    /// </summary>
    public static string SetBy(VehicleAutopilotState state, string key)
    {
        bool chased = state.TargetId.Length > 0 && state.ChaseDriven;
        // A plane target sets the plane and the crossing, and leaves the orbit in it to the player.
        AscentPlaneTarget? plane = state.TargetId.Length == 0 && state.ChaseDriven ? state.PlaneTarget : null;
        switch (key)
        {
            case nameof(VehicleAutopilotState.IncDeg):
            case nameof(VehicleAutopilotState.LaunchDescending):
                return chased ? "Set from the target." : plane != null ? "Set for " + plane.Label + "." : "";
            case nameof(VehicleAutopilotState.PeKm):
            case nameof(VehicleAutopilotState.ApKm):
            case nameof(VehicleAutopilotState.ArgPeDeg):
            case nameof(VehicleAutopilotState.ArgPeFixed):
                return chased ? "Set from the target." : "";
            case nameof(VehicleAutopilotState.LanDeg):
                return chased ? "Set from the target." : plane != null ? "Set for " + plane.Label + "." : "Set from the launch site.";
            case nameof(VehicleAutopilotState.SixDofThrottleFloor):
                return state.SixDofFloorAuto ? "Set from the craft." : "";
            // KsaVehicleHeight writes it from the colliders while the measurement is on.
            case nameof(VehicleAutopilotState.VehicleHeightM):
                return state.VehicleHeightAuto ? "Measured from the craft." : "";
            default:
                return "";
        }
    }

    public static int CountChanged(VehicleAutopilotState state, SettingSpec[]? doubles, BoolSpec[]? bools = null)
    {
        int count = 0;
        if (doubles != null)
            foreach (SettingSpec spec in doubles)
                if (TryGetDouble(state, spec.Key, out double v) && SettingMath.IsChanged(spec, v) && SetBy(state, spec.Key).Length == 0)
                    count++;
        if (bools != null)
            foreach (BoolSpec spec in bools)
                if (TryGetBool(state, spec.Key, out bool b) && SettingMath.IsChanged(spec, b) && SetBy(state, spec.Key).Length == 0)
                    count++;
        return count;
    }

    /// <summary>Writes the defaults of the given values. A value that <paramref name="locked"/> names is left as it is, the same as a value another writer owns.</summary>
    public static void ResetToDefaults(VehicleAutopilotState state, SettingSpec[]? doubles, BoolSpec[]? bools = null,
                                       Func<string, bool>? locked = null)
    {
        if (doubles != null)
            foreach (SettingSpec spec in doubles)
                if (SetBy(state, spec.Key).Length == 0 && locked?.Invoke(spec.Key) != true)
                    SetDouble(state, spec.Key, spec.Default);
        if (bools != null)
            foreach (BoolSpec spec in bools)
                if (SetBy(state, spec.Key).Length == 0 && locked?.Invoke(spec.Key) != true)
                    SetBool(state, spec.Key, spec.Default);
    }
}
