using System;
using System.Collections.Generic;
using System.Linq.Expressions;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// Reads and writes the guidance fields a <see cref="SettingSpec"/> names, so a page can count and
/// reset its changed values without a hand-written list of fields. Compiled typed delegates avoid
/// per-frame reflection and value-type boxing.
/// </summary>
internal static class SettingAccess
{
    private enum FieldKind : byte { Missing, Double, Int, Bool }

    // Reference type rather than a value-type struct so dictionary lookups copy an 8-byte reference instead of 40 bytes of delegates.
    private sealed class Accessor
    {
        public FieldKind Kind;
        public Func<VehicleAutopilotState, double>? GetDouble;
        public Action<VehicleAutopilotState, double>? SetDouble;
        public Func<VehicleAutopilotState, bool>? GetBool;
        public Action<VehicleAutopilotState, bool>? SetBool;
    }

    private static readonly Dictionary<string, Accessor> _accessors = new(StringComparer.Ordinal);

    private static Accessor Resolve(string key)
    {
        if (_accessors.TryGetValue(key, out Accessor? cached))
            return cached;

        FieldInfo? field = typeof(VehicleAutopilotState).GetField(key, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        var acc = new Accessor();
        if (field != null)
        {
            ParameterExpression stateParam = Expression.Parameter(typeof(VehicleAutopilotState), "s");
            MemberExpression member = Expression.Field(stateParam, field);
            if (field.FieldType == typeof(double))
            {
                ParameterExpression valParam = Expression.Parameter(typeof(double), "v");
                acc.Kind = FieldKind.Double;
                acc.GetDouble = Expression.Lambda<Func<VehicleAutopilotState, double>>(member, stateParam).Compile();
                acc.SetDouble = Expression.Lambda<Action<VehicleAutopilotState, double>>(Expression.Assign(member, valParam), stateParam, valParam).Compile();
            }
            else if (field.FieldType == typeof(int))
            {
                ParameterExpression valParam = Expression.Parameter(typeof(double), "v");
                MethodInfo round = typeof(Math).GetMethod(nameof(Math.Round), new[] { typeof(double) })!;
                Expression rounded = Expression.Convert(Expression.Call(round, valParam), typeof(int));
                acc.Kind = FieldKind.Int;
                acc.GetDouble = Expression.Lambda<Func<VehicleAutopilotState, double>>(Expression.Convert(member, typeof(double)), stateParam).Compile();
                acc.SetDouble = Expression.Lambda<Action<VehicleAutopilotState, double>>(Expression.Assign(member, rounded), stateParam, valParam).Compile();
            }
            else if (field.FieldType == typeof(bool))
            {
                ParameterExpression valParam = Expression.Parameter(typeof(bool), "v");
                acc.Kind = FieldKind.Bool;
                acc.GetBool = Expression.Lambda<Func<VehicleAutopilotState, bool>>(member, stateParam).Compile();
                acc.SetBool = Expression.Lambda<Action<VehicleAutopilotState, bool>>(Expression.Assign(member, valParam), stateParam, valParam).Compile();
            }
        }
        if (acc.Kind == FieldKind.Missing)
            LogHelper.WarnOnce("setting-access:" + key, $"[AFC] Setting '{key}' has no double, int or bool field on VehicleAutopilotState.");
        _accessors[key] = acc;
        return acc;
    }

    public static bool TryGetDouble(VehicleAutopilotState state, string key, out double value)
    {
        Accessor acc = Resolve(key);
        if (acc.GetDouble == null)
        {
            value = double.NaN;
            return false;
        }
        value = acc.GetDouble(state);
        return true;
    }

    public static void SetDouble(VehicleAutopilotState state, string key, double value)
    {
        Resolve(key).SetDouble?.Invoke(state, value);
    }

    public static bool TryGetBool(VehicleAutopilotState state, string key, out bool value)
    {
        Accessor acc = Resolve(key);
        if (acc.GetBool == null)
        {
            value = false;
            return false;
        }
        value = acc.GetBool(state);
        return true;
    }

    public static void SetBool(VehicleAutopilotState state, string key, bool value)
    {
        Resolve(key).SetBool?.Invoke(state, value);
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

    /// <summary>Counts the values that differ from their default on the body the craft follows (BodySettings), and that no other writer owns.</summary>
    public static int CountChanged(VehicleAutopilotState state, SettingSpec[]? doubles, BoolSpec[]? bools = null)
    {
        int count = 0;
        if (doubles != null)
            foreach (SettingSpec spec in doubles)
                if (TryGetDouble(state, spec.Key, out double v) && SettingMath.IsChanged(BodyDefaults.On(spec, state.DefaultsBody), v) && SetBy(state, spec.Key).Length == 0)
                    count++;
        if (bools != null)
            foreach (BoolSpec spec in bools)
                if (TryGetBool(state, spec.Key, out bool b) && SettingMath.IsChanged(BodyDefaults.On(spec, state.DefaultsBody), b) && SetBy(state, spec.Key).Length == 0)
                    count++;
        return count;
    }

    /// <summary>Writes the defaults of the given values, those of the body the craft follows. A value that <paramref name="locked"/> names is left as it is, the same as a value another writer owns.</summary>
    public static void ResetToDefaults(VehicleAutopilotState state, SettingSpec[]? doubles, BoolSpec[]? bools = null,
                                       Func<string, bool>? locked = null)
    {
        if (doubles != null)
            foreach (SettingSpec spec in doubles)
                if (SetBy(state, spec.Key).Length == 0 && locked?.Invoke(spec.Key) != true)
                    SetDouble(state, spec.Key, BodyDefaults.DefaultOf(spec, state.DefaultsBody));
        if (bools != null)
            foreach (BoolSpec spec in bools)
                if (SetBy(state, spec.Key).Length == 0 && locked?.Invoke(spec.Key) != true)
                    SetBool(state, spec.Key, BodyDefaults.DefaultOf(spec, state.DefaultsBody));
    }
}
