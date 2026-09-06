using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Project assembly-frame vectors onto the control axes used by ThrusterController.RecomputeDynamicData. Control From Here can rotate these axes relative to the assembly.</summary>
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

    /// <summary>Read Vehicle.Ctrl2Body because FlightComputer.ComputeControl sets its own frame on the worker and FlightComputer.CopyFrom does not copy that frame back.</summary>
    public static RcsCtrlFrame For(Vehicle vehicle)
        => new(floatQuat.Pack(vehicle.Ctrl2Body));

    public float3 ToCtrl(float3 asmb)
        => new(float3.Dot(asmb, _xAsmb), float3.Dot(asmb, _yAsmb), float3.Dot(asmb, _zAsmb));
}
