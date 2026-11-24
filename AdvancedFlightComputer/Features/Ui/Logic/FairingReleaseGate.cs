namespace AdvancedFlightComputer.Features.Ui.Logic;

// Samples in sim time while the vehicle climbs. In air the gate opens after a dwell in which the dynamic pressure
// either falls from the highest value seen since sampling began or stays below a few pascals. So the rising
// pressure after lift-off never opens it, and a craft that loads or is armed past max Q opens it on the next fall,
// or at once when it is already out of the air, without a remembered peak.
internal sealed class FairingReleaseGate
{
    private const double FallSeconds = 0.25;
    // Below this the pressure is no load on a fairing, and too small to show a fall.
    private const double ThinAirPa = 10.0;
    private double _lastTime = double.NaN, _lastQ, _peakQ, _openSince = double.NaN;

    internal const double PadClearanceM = 20.0;

    internal void Reset()
    {
        _lastTime = _openSince = double.NaN;
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
        bool first = !double.IsFinite(_lastTime) || now < _lastTime;
        if (first)
        {
            _peakQ = _lastQ = q;
            _openSince = double.NaN;
        }
        if (first || now > _lastTime)
        {
            _peakQ = Math.Max(_peakQ, q);
            bool falling = !first && q <= _lastQ && _peakQ - q >= Math.Max(ThinAirPa, 0.01 * _peakQ);
            if (!falling && q >= ThinAirPa)
                _openSince = double.NaN;
            else if (double.IsNaN(_openSince))
                _openSince = now;
            _lastQ = q;
        }
        _lastTime = now;
        return now - _openSince >= FallSeconds && q <= pressureCeilingPa
            && clearanceM >= Math.Max(PadClearanceM, altitudeFloorM);
    }
}
