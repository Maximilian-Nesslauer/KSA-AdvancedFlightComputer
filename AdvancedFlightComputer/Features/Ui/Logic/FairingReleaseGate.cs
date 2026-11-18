namespace AdvancedFlightComputer.Features.Ui.Logic;

// Sim-time samples since leaving the pad. A low first pressure sample after loading
// an ascent is not evidence that it has passed max Q.
internal sealed class FairingReleaseGate
{
    private const double FallSeconds = 0.25;
    private double _lastTime = double.NaN, _lastQ, _lowQ, _peakQ, _fallSince = double.NaN;
    private bool _hasRise;

    internal const double PadClearanceM = 20.0;

    internal void Reset()
    {
        _lastTime = _fallSince = double.NaN;
        _hasRise = false;
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
            _hasRise = false;
            _fallSince = double.NaN;
            _lowQ = _peakQ = _lastQ = q;
        }
        else if (now > _lastTime)
        {
            _lowQ = Math.Min(_lowQ, q);
            if (q > _peakQ)
            {
                _peakQ = q;
                _hasRise |= q - _lowQ >= Math.Max(10.0, 0.01 * q);
            }
            double margin = Math.Max(10.0, 0.01 * _peakQ);
            if (!_hasRise || _peakQ - q < margin || q > _lastQ)
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
