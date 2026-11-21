#nullable disable

namespace AdvancedFlightComputer.Features.AeroControl;

using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// Commands a vehicle's aerodynamic control surfaces (ControlSurfaces) for a torque, and says how much torque they can give, the way KsaRcsTorqueControl does for the thrusters. Separate from the stock flight computer, which allocates torque to gimbals and RCS only, and usable by any mode.
///
/// THE CONDITION. A surface's torque is the air's, so it follows the flight: q S L times its moment table (KsaAeroSweep.SurfaceTable) at the Mach, alpha and beta flown now, read at every deflection swept, then:
///  - divided by the game's moment limiter, 1 + k q (BodyAero6Dof.MomentLimiterPerPa), which the integrator applies to the air's whole moment;
///  - moved from the centre of mass the tables were swept about to the present one, with the surface's own force.
/// Its force is q S times its force table. The tables cover the body table's range, near engine first (within 20 deg in alpha and beta); outside it the surfaces report no authority, rather than a guess.
///
/// Main thread: it reads the craft's state and commands the surfaces through their provider.
/// </summary>
internal static class KsaAeroTorqueControl
{
    /// <summary>The air's dynamic pressure under which the surfaces count as giving nothing, Pa.</summary>
    private const double MinQPa = 50.0;

    /// <summary>The flight condition the surfaces were read at, and the last command: surfaces commanded, the torque they give and how far short of the demand, the force with it (frame of the command), Mach, q, alpha and beta (deg), and whether that was inside the tables.</summary>
    internal readonly record struct Report(
        int Surfaces, double3 GivenNm, double ShortfallNm, double3 ForceN,
        double Mach, double QPa, double AlphaDeg, double BetaDeg, bool InTable,
        double[] CommandDeg = null, double[] NowDeg = null);

    /// <summary>What the best RCS thruster's torque costs against the surfaces' shortfall, per N m (a shortfall costs one at priority 1, a degree of surface about a thousandth): the RCS is spent before any demand goes unmet, and only on what the surfaces cannot give.</summary>
    private const double BackupPricePerNm = 0.02;

    private sealed class Slot
    {
        public readonly List<ControlSurface> Surfaces = new();
        public Report Last;
        /// <summary>The deflections commanded last, for the smoothing (Command's smoothS).</summary>
        public double[] Commanded;
    }

    private static readonly ConditionalWeakTable<Vehicle, Slot> Slots = new();

    private static Slot SlotOf(Vehicle vehicle) => Slots.GetValue(vehicle, static _ => new Slot());

    /// <summary>
    /// The allocator for the vehicle's surfaces as the air is now, in the frame of the asmb unit vectors <paramref name="ax"/>, <paramref name="ay"/>, <paramref name="az"/>: one surface per entry of <paramref name="surfaces"/>, which this fills. Null, with the condition in <paramref name="condition"/>, when there is nothing to steer with: no surfaces, no tables for them, too little air, or a flight outside the tables.
    /// </summary>
    internal static AeroTorqueAllocator Measure(Vehicle vehicle, KsaAeroSweep.Result aero, double3 ax, double3 ay, double3 az,
                                                double momentLimiterPerPa, List<ControlSurface> surfaces, out Report condition)
    {
        condition = default;
        surfaces.Clear();
        if (vehicle == null || aero?.Surfaces == null || aero.Surfaces.Length == 0 || aero.Atmosphere == null
            || !(aero.ReferenceArea > 0.0) || !(aero.ReferenceLength > 0.0))
            return null;

        float3 airF = PhysicsStates.ComputeAirVelocityBody(vehicle.BubbleOrigin, vehicle.KinematicStates, vehicle.PhysicsEnvironment);
        double3 air = double3.Unpack(in airF);
        double speed = air.Length();
        double q = 0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * speed * speed;
        if (!(speed > 1.0))
            return null;
        // Retrograde-first, as the body table: u is the craft's velocity through the air in body axes, alpha = atan2(u_z, -u_x), beta = atan2(u_y, -u_x). Nose first (an ascent, #216), about the nose instead, from the surfaces' nose-first tables where the sweep has them.
        double3 u = air / speed;
        // Engine first, nose first (an ascent) or belly first (an orbital entry): the surfaces' tables for whichever set the air is in (KsaAeroSweep.TryPickTables).
        bool inTable = KsaAeroSweep.TryPickTables(aero, u, surfacesToo: true, out KsaAeroSweep.TableSet set, out double alpha, out double beta);
        double mach = aero.Atmosphere.Mach(speed);
        condition = new Report(0, default, 0.0, default, mach, q, alpha * 180.0 / Math.PI, beta * 180.0 / Math.PI, inTable);
        if (!inTable || !(q > MinQPa))
            return null;

        var live = new List<ControlSurface>();
        ControlSurfaces.Collect(vehicle, live);
        double divisor = 1.0 + (double.IsFinite(momentLimiterPerPa) && momentLimiterPerPa > 0.0 ? momentLimiterPerPa * q : 0.0);
        float3 comNow = vehicle.CenterOfMassAsmbF;
        double3 shift = aero.BodyMomentComAsmb - new double3(comNow.X, comNow.Y, comNow.Z);
        AeroTorqueAllocator allocator = Build(aero, live, mach, q, alpha, beta, divisor, shift, ax, ay, az, surfaces, set);
        condition = condition with { Surfaces = surfaces.Count };
        return allocator;
    }

    /// <summary>
    /// The allocator for the surfaces in <paramref name="live"/> that the sweep has tables for, at a given flight condition: Mach, dynamic pressure (Pa), alpha and beta (radians, retrograde-first), the moment limiter's divisor, and the shift from the centre of mass the tables were swept about to the present one (asmb, m). In the frame of the asmb unit vectors <paramref name="ax"/>, <paramref name="ay"/>, <paramref name="az"/>; <paramref name="surfaces"/> gets one surface per allocator entry. Null where none has a table. Reads nothing of the craft, so a check can ask it at any condition.
    /// </summary>
    internal static AeroTorqueAllocator Build(KsaAeroSweep.Result aero, IReadOnlyList<ControlSurface> live,
                                              double mach, double q, double alpha, double beta, double divisor, double3 comShift,
                                              double3 ax, double3 ay, double3 az, List<ControlSurface> surfaces, bool noseFirst = false)
        => Build(aero, live, mach, q, alpha, beta, divisor, comShift, ax, ay, az, surfaces, noseFirst ? KsaAeroSweep.TableSet.Nose : KsaAeroSweep.TableSet.Tail);

    /// <summary>The same from a table set's surface tables (KsaAeroSweep.TableSet), alpha and beta about that set's frame.</summary>
    internal static AeroTorqueAllocator Build(KsaAeroSweep.Result aero, IReadOnlyList<ControlSurface> live,
                                              double mach, double q, double alpha, double beta, double divisor, double3 comShift,
                                              double3 ax, double3 ay, double3 az, List<ControlSurface> surfaces, KsaAeroSweep.TableSet set)
    {
        surfaces.Clear();
        if (aero?.Surfaces == null || !(aero.ReferenceArea > 0.0) || !(aero.ReferenceLength > 0.0))
            return null;
        double qs = q * aero.ReferenceArea, qsl = qs * aero.ReferenceLength;
        double3 shift = comShift;
        divisor = divisor > 0.0 && double.IsFinite(divisor) ? divisor : 1.0;
        var rows = new List<(ControlSurface Surface, KsaAeroSweep.SurfaceTable Table)>();
        foreach (ControlSurface surface in live)
            foreach (KsaAeroSweep.SurfaceTable table in aero.Surfaces)
                if (ReferenceEquals(table.Part, surface.Part) && table.DeflectionDeg.Length > 1
                    && table.ForceTables(set).Length == table.DeflectionDeg.Length)
                {
                    rows.Add((surface, table));
                    break;
                }
        if (rows.Count == 0)
            return null;

        var allocator = new AeroTorqueAllocator(rows.Count);
        for (int i = 0; i < rows.Count; i++)
        {
            KsaAeroSweep.SurfaceTable t = rows[i].Table;
            int n = t.DeflectionDeg.Length;
            var torque = new double[3 * n];
            var force = new double[3 * n];
            for (int k = 0; k < n; k++)
            {
                AdvancedFlightComputer.Guidance.Numerics.Flight.BodyAeroTable ft = t.ForceTables(set)[k], mt = t.MomentTables(set)[k];
                double mk = Math.Clamp(mach, ft.MachMin, ft.MachMax);
                ft.Coefficients(mk, alpha, beta, out double cx, out double cy, out double cz);
                mt.Coefficients(mk, alpha, beta, out double cl, out double cm, out double cn);
                var f = new double3(qs * cx, qs * cy, qs * cz);
                // About the present centre of mass, M_now = M_swept + (r_swept - r_now) x F, then the game's limiter on the whole of it.
                double3 m = (new double3(qsl * cl, qsl * cm, qsl * cn) + double3.Cross(shift, f)) / divisor;
                torque[3 * k] = double3.Dot(m, ax); torque[3 * k + 1] = double3.Dot(m, ay); torque[3 * k + 2] = double3.Dot(m, az);
                force[3 * k] = double3.Dot(f, ax); force[3 * k + 1] = double3.Dot(f, ay); force[3 * k + 2] = double3.Dot(f, az);
            }
            allocator.SetSurface(i, t.DeflectionDeg, torque, force);
            surfaces.Add(rows[i].Surface);
        }
        return allocator;
    }

    /// <summary>
    /// What the surfaces can give about each axis of the frame (ax, ay, az) each way, as the air is now: six entries, [2a] positive and [2a + 1] negative about axis a. False, all zero, where Measure finds nothing to steer with.
    /// </summary>
    internal static bool TryCharacterise(Vehicle vehicle, KsaAeroSweep.Result aero, double3 ax, double3 ay, double3 az,
                                         double momentLimiterPerPa, Span<AeroTorqueAllocator.AxisAuthority> six)
    {
        six.Clear();
        AeroTorqueAllocator allocator = Measure(vehicle, aero, ax, ay, az, momentLimiterPerPa, new List<ControlSurface>(), out _);
        if (allocator == null)
            return false;
        for (int a = 0; a < 3; a++)
        {
            six[2 * a] = allocator.Characterise(a, 1);
            six[2 * a + 1] = allocator.Characterise(a, -1);
        }
        return true;
    }

    /// <summary>
    /// Commands the surfaces for a torque (asmb axes, N m), allocated in the frame (mx, my, mz) with <paramref name="priority"/> weighting each of its axes' shortfall. With nothing to steer with, every surface is sent home. Returns what was given.
    /// </summary>
    /// <param name="stepS">The time to the next command, s: each surface is asked only for what its rate reaches by then, from where it is now (AeroTorqueAllocator.Allocate). Not positive: anywhere in its range.</param>
    /// <param name="backup">The RCS thrusters in the frame (mx, my, mz) (KsaRcsTorqueControl.Measure), to share the demand with in one programme: the surfaces then take the axes the RCS finds dear. The RCS is not fired here; the caller gives it what the surfaces leave.</param>
    /// <param name="smoothS">With a backup that can give torque: the time the deflections follow the allocator's answer over, s, a first-order lag from the last command (dynamic control allocation: their rate of movement priced, not their deflection); the backup then takes what they give short. 0: none.</param>
    internal static Report Command(Vehicle vehicle, KsaAeroSweep.Result aero, double3 torqueAsmb,
                                   double3 mx, double3 my, double3 mz, ReadOnlySpan<double> priority, double momentLimiterPerPa,
                                   double stepS = 0.0, RcsTorqueAllocator backup = null, double smoothS = 0.0)
    {
        Slot slot = SlotOf(vehicle);
        AeroTorqueAllocator allocator = Measure(vehicle, aero, mx, my, mz, momentLimiterPerPa, slot.Surfaces, out Report condition);
        if (allocator == null)
        {
            Release(vehicle);
            slot.Last = condition with { ShortfallNm = torqueAsmb.Length() };
            return slot.Last;
        }
        double dx = double3.Dot(torqueAsmb, mx), dy = double3.Dot(torqueAsmb, my), dz = double3.Dot(torqueAsmb, mz);
        int count = allocator.Count;
        var deflection = new double[count];
        var now = new double[count];
        var reach = new double[count];
        for (int i = 0; i < count; i++)
        {
            now[i] = slot.Surfaces[i].DeflectionDeg;
            reach[i] = stepS > 0.0 && slot.Surfaces[i].RateDegS > 0.0 ? slot.Surfaces[i].RateDegS * stepS : 0.0;
        }
        BackupColumns(backup, out double[] backupTorque, out double[] backupCost);
        AeroTorqueAllocator.Result r = allocator.Allocate(dx, dy, dz, priority, deflection, now, reach,
                                                          backupTorque, backupCost, new double[backupCost.Length]);
        if (!r.Solved)
        {
            Release(vehicle);
            slot.Last = condition with { ShortfallNm = torqueAsmb.Length() };
            return slot.Last;
        }
        if (smoothS > 0.0 && stepS > 0.0 && backup != null && backup.Count > 0)
        {
            // The surfaces follow the answer through a lag, from where they were last sent; what they then give is the truth the backup is told.
            double k = stepS / (smoothS + stepS);
            bool known = slot.Commanded != null && slot.Commanded.Length == count;
            for (int i = 0; i < count; i++)
            {
                double from = known ? slot.Commanded[i] : now[i];
                double want = from + k * (deflection[i] - from);
                if (reach[i] > 0.0)
                    want = Math.Clamp(want, now[i] - reach[i], now[i] + reach[i]);
                deflection[i] = want;
            }
            r = allocator.Evaluate(deflection, dx, dy, dz);
        }
        slot.Commanded = (double[])deflection.Clone();
        for (int i = 0; i < slot.Surfaces.Count; i++)
            slot.Surfaces[i].Command(deflection[i]);
        slot.Last = condition with
        {
            GivenNm = new double3(r.TorqueX, r.TorqueY, r.TorqueZ),
            ShortfallNm = r.ShortfallNm,
            ForceN = new double3(r.ForceX, r.ForceY, r.ForceZ),
            CommandDeg = deflection,
            NowDeg = now,
        };
        return slot.Last;
    }

    /// <summary>The RCS thrusters as backup columns for the surfaces' programme: each one's torque at full duty, and its propellant priced so the best of them costs BackupPricePerNm a newton metre. Empty without a backup.</summary>
    private static void BackupColumns(RcsTorqueAllocator backup, out double[] torque, out double[] cost)
    {
        if (backup == null || backup.Count == 0)
        {
            torque = [];
            cost = [];
            return;
        }
        int m = backup.Count;
        torque = new double[3 * m];
        cost = new double[m];
        double best = double.PositiveInfinity;
        for (int i = 0; i < m; i++)
        {
            double t = Math.Sqrt(backup.Torque[3 * i] * backup.Torque[3 * i] + backup.Torque[3 * i + 1] * backup.Torque[3 * i + 1]
                                 + backup.Torque[3 * i + 2] * backup.Torque[3 * i + 2]);
            double flow = backup.Flow[i];
            if (flow > 0.0 && double.IsFinite(flow) && t > 0.0 && double.IsFinite(t))
                best = Math.Min(best, flow / t);
        }
        if (!double.IsFinite(best))
        {
            torque = [];
            cost = [];
            return;
        }
        double perKgS = BackupPricePerNm / best;
        for (int i = 0; i < m; i++)
        {
            double flow = backup.Flow[i];
            bool usable = flow > 0.0 && double.IsFinite(flow);
            for (int r = 0; r < 3; r++)
                torque[3 * i + r] = usable ? backup.Torque[3 * i + r] : 0.0;
            cost[i] = usable ? flow * perKgS : double.NaN;
        }
    }

    /// <summary>Sends every surface on the vehicle home. Main thread.</summary>
    internal static void Release(Vehicle vehicle)
    {
        if (vehicle == null || !ControlSurfaces.Any)
            return;
        var surfaces = new List<ControlSurface>();
        ControlSurfaces.Collect(vehicle, surfaces);
        foreach (ControlSurface s in surfaces)
            s.Command(0.0);
    }

    /// <summary>The last command's report for this vehicle.</summary>
    internal static Report Last(Vehicle vehicle) => vehicle != null && Slots.TryGetValue(vehicle, out Slot slot) ? slot.Last : default;
}
