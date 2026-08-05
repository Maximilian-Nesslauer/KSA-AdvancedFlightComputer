using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// The vehicle's control frame expressed as three assembly-frame axes, the
/// same decomposition <see cref="ThrusterController.RecomputeDynamicData"/>
/// builds. Everything this feature computes lives in the control frame, not
/// the assembly frame: the stock actuator data it reads back
/// (<c>ThrusterControllerState.IntendedForce</c>, <c>ControlMap</c>,
/// <c>FlightComputer.RcsTorqueAuthority</c>, <c>ErrorAngles</c>) is expressed
/// there, and so are the translation commands the player issues. The two
/// frames coincide until a control point is selected through "Control From
/// Here", at which point an assembly-frame impulse would name different
/// thrusters than the ones stock would fire.
/// </summary>
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

    /// <summary>Reads the frame off the vehicle, not off its FlightComputer:
    /// <c>FlightComputer.Ctrl2Asmb</c> is set by the worker inside
    /// <c>ComputeControl</c> and <c>CopyFrom</c> does not carry it back, so the
    /// vehicle's own FlightComputer holds a stale identity on the main
    /// thread.</summary>
    public static RcsCtrlFrame For(Vehicle vehicle)
        => new(floatQuat.Pack(vehicle.Ctrl2Body));

    public float3 ToCtrl(float3 asmb)
        => new(float3.Dot(asmb, _xAsmb), float3.Dot(asmb, _yAsmb), float3.Dot(asmb, _zAsmb));
}
