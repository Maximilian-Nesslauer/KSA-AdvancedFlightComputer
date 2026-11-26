using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// An atmosphere given as density and pressure samples at increasing altitudes from sea level to the top, with ln(rho) and ln(P) interpolated linearly between the samples.
///
/// The mod fills it from the game's own PhysicalAtmosphere.ComputeDensityAtAltitude and ComputePressureAtAltitude, so the solvers fly the game's layered air on a worker thread without a reference to the game. Inside an isothermal layer ln(rho) and ln(P) are straight lines in geopotential, which is nearly linear in altitude over one sample, so the interpolation is nearly exact there; inside a layer with a lapse rate it is a chord of a smooth curve. The slope of ln(rho) jumps where the lapse rate changes, so the caller puts a sample on every point of the temperature profile and no chord straddles a corner. The adapter that builds the model measures the remaining error against the game.
///
/// The slope that <see cref="Density(Dual)"/> gives is constant between two samples, so the local scale height (<see cref="AtmosphereModel.ScaleHeightAt"/>) is the game's density ratio across one interval: dh / ln(rho_i / rho_i+1).
/// </summary>
public sealed class SampledAtmosphere : AtmosphereModel
{
    private readonly double[] _altitude;
    private readonly double[] _lnDensity;
    private readonly double[] _lnPressure;
    private readonly double[] _lnSpeedOfSound;
    private readonly double[]? _lnDynamicViscosity;
    private readonly double[] _density;
    private readonly double[] _pressure;
    private readonly double[] _speedOfSoundSamples;
    private readonly double[]? _dynamicViscositySamples;
    private readonly double _topAltitude;
    private readonly double _speedOfSound;
    private readonly double _topSpeedOfSound;
    private readonly double _topDynamicViscosity;

    // A uniform index over the samples: _bucketStart[k] is the last sample at or below k * _bucketWidth, so a lookup starts there and steps forward over the few samples inside one bucket.
    private readonly double _bucketWidth;
    private readonly int[] _bucketStart;

    public override double SeaLevelDensity => _density[0];

    public override double SeaLevelPressure => _pressure[0];

    public override double TopAltitude => _topAltitude;

    public override double SpeedOfSound => _speedOfSound;

    /// <summary>The sample altitudes, m, from zero up. A copy.</summary>
    public double[] AltitudeSamples => (double[])_altitude.Clone();

    /// <summary>The density samples, kg/m^3. A copy.</summary>
    public double[] DensitySamples => (double[])_density.Clone();

    /// <summary>The pressure samples, Pa. A copy.</summary>
    public double[] PressureSamples => (double[])_pressure.Clone();

    /// <summary>The local speed-of-sound samples, m/s. A copy.</summary>
    public double[] SpeedOfSoundSamples => (double[])_speedOfSoundSamples.Clone();

    /// <summary>The dynamic-viscosity samples, Pa s, or null if viscosity was not supplied. A copy.</summary>
    public double[]? DynamicViscositySamples => _dynamicViscositySamples != null ? (double[])_dynamicViscositySamples.Clone() : null;

    /// <param name="altitude">Sample altitudes, m: the first is zero, the rest strictly increase, and the last is at or above <paramref name="topAltitude"/>.</param>
    /// <param name="density">Density at each altitude, kg/m^3, all finite and positive.</param>
    /// <param name="pressure">Pressure at each altitude, Pa, all finite and positive.</param>
    /// <param name="topAltitude">Altitude above which density and pressure are exactly zero, m.</param>
    /// <param name="speedOfSound">Sea-level reference speed of sound, m/s.</param>
    /// <param name="speedOfSoundSamples">Optional local speed of sound at each altitude, m/s. When omitted, ideal-gas scaling from P/rho is used.</param>
    /// <param name="dynamicViscositySamples">Optional dynamic viscosity at each altitude, Pa s.</param>
    public SampledAtmosphere(double[] altitude, double[] density, double[] pressure, double topAltitude, double speedOfSound,
                             double[]? speedOfSoundSamples = null, double[]? dynamicViscositySamples = null)
    {
        ArgumentNullException.ThrowIfNull(altitude);
        ArgumentNullException.ThrowIfNull(density);
        ArgumentNullException.ThrowIfNull(pressure);
        int n = altitude.Length;
        if (n < 2 || density.Length != n || pressure.Length != n)
            throw new ArgumentException("Altitude, density and pressure need the same number of samples, at least two.", nameof(altitude));
        if (speedOfSoundSamples != null && speedOfSoundSamples.Length != n)
            throw new ArgumentException("Speed-of-sound samples must match altitude sample count.", nameof(speedOfSoundSamples));
        if (dynamicViscositySamples != null && dynamicViscositySamples.Length != n)
            throw new ArgumentException("Dynamic-viscosity samples must match altitude sample count.", nameof(dynamicViscositySamples));
        if (altitude[0] != 0.0)
            throw new ArgumentOutOfRangeException(nameof(altitude), altitude[0], "The first sample must be at sea level.");
        if (!(topAltitude > 0.0) || !(topAltitude <= altitude[n - 1]))
            throw new ArgumentOutOfRangeException(nameof(topAltitude), topAltitude, "The top must be positive and inside the samples.");
        if (!(speedOfSound > 0.0) || !double.IsFinite(speedOfSound))
            throw new ArgumentOutOfRangeException(nameof(speedOfSound), speedOfSound, "The speed of sound must be finite and positive.");

        _altitude = (double[])altitude.Clone();
        _density = (double[])density.Clone();
        _pressure = (double[])pressure.Clone();
        _lnDensity = new double[n];
        _lnPressure = new double[n];
        _speedOfSoundSamples = new double[n];
        _lnSpeedOfSound = new double[n];
        if (dynamicViscositySamples != null)
        {
            _dynamicViscositySamples = new double[n];
            _lnDynamicViscosity = new double[n];
        }
        _topAltitude = topAltitude;
        _speedOfSound = speedOfSound;
        double slPoverRho = pressure[0] / density[0];
        for (int i = 0; i < n; i++)
        {
            if (!double.IsFinite(altitude[i]) || (i > 0 && !(altitude[i] > altitude[i - 1])))
                throw new ArgumentException($"Sample altitude {i} is not finite and above the sample below it.", nameof(altitude));
            if (!(density[i] > 0.0) || !double.IsFinite(density[i]) || !(pressure[i] > 0.0) || !double.IsFinite(pressure[i]))
                throw new ArgumentOutOfRangeException(nameof(density), $"Sample {i} is not finite and positive (rho {density[i]}, P {pressure[i]}).");
            _lnDensity[i] = Math.Log(density[i]);
            _lnPressure[i] = Math.Log(pressure[i]);

            double ai = speedOfSoundSamples != null
                ? speedOfSoundSamples[i]
                : speedOfSound * Math.Sqrt((pressure[i] / density[i]) / slPoverRho);
            if (!(ai > 0.0) || !double.IsFinite(ai))
                throw new ArgumentOutOfRangeException(nameof(speedOfSoundSamples), $"Speed-of-sound sample {i} is not finite and positive ({ai}).");
            _speedOfSoundSamples[i] = ai;
            _lnSpeedOfSound[i] = Math.Log(ai);

            if (dynamicViscositySamples != null)
            {
                double mu = dynamicViscositySamples[i];
                if (!(mu > 0.0) || !double.IsFinite(mu))
                    throw new ArgumentOutOfRangeException(nameof(dynamicViscositySamples), $"Dynamic-viscosity sample {i} is not finite and positive ({mu}).");
                _dynamicViscositySamples![i] = mu;
                _lnDynamicViscosity![i] = Math.Log(mu);
            }
        }

        int buckets = n;
        _bucketWidth = _altitude[n - 1] / buckets;
        _bucketStart = new int[buckets + 1];
        int j = 0;
        for (int k = 0; k <= buckets; k++)
        {
            double at = k * _bucketWidth;
            while (j < n - 2 && _altitude[j + 1] <= at)
                j++;
            _bucketStart[k] = j;
        }

        _topSpeedOfSound = _topAltitude < _altitude[n - 1]
            ? Interpolate(_lnSpeedOfSound, _topAltitude)
            : _speedOfSoundSamples[n - 1];
        _topDynamicViscosity = _lnDynamicViscosity == null
            ? 0.0
            : (_topAltitude < _altitude[n - 1]
                ? Interpolate(_lnDynamicViscosity, _topAltitude)
                : _dynamicViscositySamples![n - 1]);
    }

    /// <summary>The interval that holds a positive altitude below the top, and the fraction across it.</summary>
    private int Interval(double altitude, out double fraction)
    {
        int k = Math.Min((int)(altitude / _bucketWidth), _bucketStart.Length - 1);
        int i = _bucketStart[k];
        while (i < _altitude.Length - 2 && _altitude[i + 1] <= altitude)
            i++;
        fraction = (altitude - _altitude[i]) / (_altitude[i + 1] - _altitude[i]);
        return i;
    }

    private double Interpolate(double[] ln, double altitude)
    {
        int i = Interval(altitude, out double t);
        return Math.Exp(ln[i] + t * (ln[i + 1] - ln[i]));
    }

    private Dual Interpolate(double[] ln, Dual altitude)
    {
        int i = Interval(altitude.V, out double t);
        double slope = (ln[i + 1] - ln[i]) / (_altitude[i + 1] - _altitude[i]);
        double value = Math.Exp(ln[i] + t * (ln[i + 1] - ln[i]));
        return new Dual(value, value * slope * altitude.D);
    }

    public override double Density(double altitude)
    {
        if (altitude >= _topAltitude)
            return 0.0;
        return altitude <= 0.0 ? _density[0] : Interpolate(_lnDensity, altitude);
    }

    public override double Pressure(double altitude)
    {
        if (altitude >= _topAltitude)
            return 0.0;
        return altitude <= 0.0 ? _pressure[0] : Interpolate(_lnPressure, altitude);
    }

    public override Dual Density(Dual altitude)
    {
        if (altitude.V >= _topAltitude)
            return new Dual(0.0, 0.0);
        return altitude.V <= 0.0 ? new Dual(_density[0], 0.0) : Interpolate(_lnDensity, altitude);
    }

    public override Dual Pressure(Dual altitude)
    {
        if (altitude.V >= _topAltitude)
            return new Dual(0.0, 0.0);
        return altitude.V <= 0.0 ? new Dual(_pressure[0], 0.0) : Interpolate(_lnPressure, altitude);
    }

    public override double SpeedOfSoundAt(double altitude)
    {
        if (altitude <= 0.0)
            return _speedOfSoundSamples[0];
        if (altitude >= _topAltitude)
            return _topSpeedOfSound;
        return Interpolate(_lnSpeedOfSound, altitude);
    }

    public override Dual SpeedOfSoundAt(Dual altitude)
    {
        if (altitude.V <= 0.0)
            return new Dual(_speedOfSoundSamples[0], 0.0);
        if (altitude.V >= _topAltitude)
            return new Dual(_topSpeedOfSound, 0.0);
        return Interpolate(_lnSpeedOfSound, altitude);
    }

    public override double DynamicViscosityAt(double altitude)
    {
        if (_lnDynamicViscosity == null)
            return 0.0;
        if (altitude <= 0.0)
            return _dynamicViscositySamples![0];
        if (altitude >= _topAltitude)
            return _topDynamicViscosity;
        return Interpolate(_lnDynamicViscosity, altitude);
    }

    public override string ToString()
        => $"rho0={SeaLevelDensity:G4} kg/m^3, P0={SeaLevelPressure:G6} Pa, "
         + $"H0={SeaLevelScaleHeight / 1000.0:F2} km, top={TopAltitude / 1000.0:F1} km, "
         + $"a={SpeedOfSound:F1} m/s, {_altitude.Length} samples";
}
