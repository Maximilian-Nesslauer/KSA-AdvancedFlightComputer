using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Project assembly-frame vectors onto the control axes used by ThrusterController.RecomputeDynamicData. Control From Here can rotate these axes relative to the assembly, and the thrust-aligned TNM frame can tilt them away from the helm.</summary>
internal readonly struct RcsCtrlFrame
{
    public readonly floatQuat Ctrl2Body;
    private readonly float3 _xAsmb;
    private readonly float3 _yAsmb;
    private readonly float3 _zAsmb;

    public RcsCtrlFrame(floatQuat ctrl2Body)
    {
        Ctrl2Body = ctrl2Body;
        _xAsmb = float3.UnitX.Transform(ctrl2Body);
        _yAsmb = float3.UnitY.Transform(ctrl2Body);
        _zAsmb = float3.UnitZ.Transform(ctrl2Body);
    }

    /// <summary>Use the frame of the stock thruster cache, ThrusterControllerGlobalState.CachedCtrl2Body, because the intended force and torque of every thruster are in these axes. A vehicle without thruster states uses the live control frame.</summary>
    public static RcsCtrlFrame For(Vehicle vehicle)
        => new(TryGetCache(vehicle, out ThrusterControllerGlobalState cache)
            ? cache.CachedCtrl2Body
            : LiveCtrl2Body(vehicle));

    /// <summary>The control frame stock selects on the worker. FlightComputer.ComputeControl uses Tnm2Asmb when ActiveControlFrame is TNM and the helm frame otherwise. FlightComputer.CopyFrom copies Tnm2Asmb and ActiveControlFrame back to the main thread, but not Ctrl2Asmb.</summary>
    public static floatQuat LiveCtrl2Body(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        return floatQuat.Pack(fc.ActiveControlFrame.IsTnm() ? fc.Tnm2Asmb : vehicle.Helm2Body);
    }

    /// <summary>True when the stock thruster cache is in the live control frame and <paramref name="ctrl2Body"/> is in the cache frame. Both checks use the stock ThrusterControllerGlobalState.IsCtrl2BodyCurrent tolerance, because Rocket.UpdateThrusterCache keeps a cached frame inside that tolerance.</summary>
    public static bool IsSettled(Vehicle vehicle, floatQuat ctrl2Body)
        => TryGetCache(vehicle, out ThrusterControllerGlobalState cache)
           && cache.IsCtrl2BodyCurrent(LiveCtrl2Body(vehicle))
           && cache.IsCtrl2BodyCurrent(ctrl2Body);

    /// <summary>True when <paramref name="ctrl2Body"/> is within the stock tolerance of the thruster cache frame.</summary>
    public static bool MatchesCache(Vehicle vehicle, floatQuat ctrl2Body)
        => TryGetCache(vehicle, out ThrusterControllerGlobalState cache)
           && cache.IsCtrl2BodyCurrent(ctrl2Body);

    /// <summary>Rotation from control axes to CCI for a frame given relative to the vehicle body.</summary>
    public static doubleQuat Ctrl2Cci(Vehicle vehicle, floatQuat ctrl2Body)
        => doubleQuat.Concatenate(doubleQuat.Unpack(in ctrl2Body), vehicle.GetBody2Cci());

    public float3 ToCtrl(float3 asmb)
        => new(float3.Dot(asmb, _xAsmb), float3.Dot(asmb, _yAsmb), float3.Dot(asmb, _zAsmb));

    private static bool TryGetCache(Vehicle vehicle, out ThrusterControllerGlobalState cache)
    {
        if (ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var stateList))
        {
            cache = stateList.GlobalState;
            return true;
        }
        cache = default;
        return false;
    }
}
