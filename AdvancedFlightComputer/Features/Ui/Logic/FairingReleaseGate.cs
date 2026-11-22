namespace AdvancedFlightComputer.Features.Ui.Logic;

// Samples in sim time while the vehicle climbs. In air the dynamic pressure must fall for a dwell from the
// highest value seen since sampling began, so the rising pressure after lift-off never opens the gate, and a
// craft that loads or is armed past max Q opens it on its next fall without a remembered peak.
internal sealed class FairingReleaseGate
{
    private const double FallSeconds = 0.25;
    private double _lastTime = double.NaN, _lastQ, _peakQ, _fallSince = double.NaN;

    internal const double PadClearanceM = 20.0;

    internal void Reset()
    {
        _lastTime = _fallSince = double.NaN;
    }

    internal bool Sample(double now, double q, double clearanceM, bool climbing, bool air,
        double altitudeFloorM, double pressureCeilingPa)
    {
        if (!climbing)
        {
            Reset();
            return false;
        }
        if (!double.IsFinite(clearanceM) || !double.IsFinite(now))
            return false;
        if (!air)
            return clearanceM >= PadClearanceM;
        if (!double.IsFinite(q) || q < 0.0)
        {
            Reset();
            return false;
        }
        if (!double.IsFinite(_lastTime) || now < _lastTime)
        {
            _fallSince = double.NaN;
            _peakQ = _lastQ = q;
        }
        else if (now > _lastTime)
        {
            _peakQ = Math.Max(_peakQ, q);
            if (_peakQ - q < Math.Max(10.0, 0.01 * _peakQ) || q > _lastQ)
                _fallSince = double.NaN;
            else if (double.IsNaN(_fallSince))
                _fallSince = now;
            _lastQ = q;
        }
        _lastTime = now;
        return now - _fallSince >= FallSeconds && q <= pressureCeilingPa
            && clearanceM >= Math.Max(PadClearanceM, altitudeFloorM);
    }
}
