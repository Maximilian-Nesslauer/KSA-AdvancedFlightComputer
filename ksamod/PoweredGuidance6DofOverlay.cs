using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// World-space overlay for the 6-DOF plan, matching the G-FOLD one so the two read
// the same way. Shares the projection plumbing in PoweredGuidanceOverlayCore.
//
// This is the instrument for "is the PLAN sensible" as opposed to "is the TRACKER
// following it" — which the tracking-error readout answers. A plan that dives at
// the ground, wanders downrange, or demands a wild attitude profile is visible
// here immediately and is not diagnosable from numbers alone.
public static partial class PoweredGuidanceWindow
{
    private static bool _show6DofOverlay = true;

    private static void Draw6DofOverlay(Viewport vp, IParentBody parent)
    {
        if (!_show6DofOverlay || !_sixDofActive)
            return;

        Ksa6DofGuidance g = _sixDof;
        if (g == null || !g.HasPlan)
            return;
        if (!SetupProjection(parent))
            return;

        // Rebuild the site frame live rather than caching it from solve time: the
        // plan is flown against the rotating body, so a stale frame slides off the
        // ground as the world turns. Same reasoning as the G-FOLD overlay.
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame f = KsaFrameBridge.BuildSiteFrame(siteCci);

        ReadOnlySpan<double> px = g.PlanState;
        ReadOnlySpan<double> pu = g.PlanControl;
        int n = g.Nodes;
        if (px.Length < n * 14)
            return;

        var trajCol = new ImColor8(120, 200, 255);
        var nodeCol = new ImColor8(190, 230, 255);
        var thrCol = new ImColor8(255, 200, 60);
        var axisCol = new ImColor8(255, 120, 220);
        var tgtCol = new ImColor8(255, 90, 220);
        var padCol = new ImColor8(235, 235, 235);
        var refCol = new ImColor8(120, 255, 150);

        ImDrawListPtr dl = BeginOverlayWindow(vp, "##sixdof_overlay");

        // Materialised up front rather than read through a local function: a Span
        // cannot be captured by one, and projecting each node once is cheaper anyway.
        var node = new double3[n];
        for (int k = 0; k < n; k++)
            node[k] = f.PosToCci(new double3(px[k * 14 + 0], px[k * 14 + 1], px[k * 14 + 2]));

        // --- Glideslope cone, drawn first so the path sits on top of it.
        //
        // Same construction as the G-FOLD overlay: apex at the target, opening
        // upward, radius cot(angle) * height. Drawing it from the SAME angle the
        // solver was configured with is the point — a cone drawn from a separate
        // number would keep looking right while the solver enforced something else.
        if (_sixDofGlideSlopeDeg > 0.0)
        {
            var coneCol = new ImColor8(90, 140, 190);
            double apexZ = _sixDofTargetAltM;
            double topZ = Math.Max(px[0 * 14 + 2], apexZ + 1.0);   // up to the plan's start
            double cot = 1.0 / Math.Tan(Math.Clamp(_sixDofGlideSlopeDeg, 1e-3, 89.999) * Math.PI / 180.0);
            double3 apex = f.PosToCci(new double3(0, 0, apexZ));

            const int rings = 4, seg = 28;
            for (int ring = 1; ring <= rings; ring++)
            {
                double dz = (topZ - apexZ) * ring / rings;
                double rad = cot * dz, z = apexZ + dz;
                double3 prev = default;
                for (int j = 0; j <= seg; j++)
                {
                    double th = 2.0 * Math.PI * j / seg;
                    double3 p = f.PosToCci(new double3(rad * Math.Cos(th), rad * Math.Sin(th), z));
                    if (j > 0) OvLine(dl, prev, p, coneCol, 1.3f);
                    prev = p;
                }
            }
            double topRad = cot * (topZ - apexZ);
            for (int a = 0; a < 4; a++)
            {
                double th = Math.PI / 2.0 * a;
                double3 rim = f.PosToCci(new double3(topRad * Math.Cos(th), topRad * Math.Sin(th), topZ));
                OvLine(dl, apex, rim, coneCol, 1.3f);
            }
        }

        // --- planned path ---
        for (int k = 0; k + 1 < n; k++)
            OvLine(dl, node[k], node[k + 1], trajCol, 2.0f);

        // --- per-node markers, thrust vectors and body axis ---
        // Thrust and attitude are drawn because they are what a 6-DOF plan adds over
        // a 3-DOF one: if the attitude profile is nonsense the path can still look
        // fine, and this is the only place that shows up.
        double thrustScale = 0.0;
        for (int k = 0; k < n; k++)
        {
            double t = Math.Sqrt(
                pu[k * 4 + 0] * pu[k * 4 + 0] +
                pu[k * 4 + 1] * pu[k * 4 + 1] +
                pu[k * 4 + 2] * pu[k * 4 + 2]);
            thrustScale = Math.Max(thrustScale, t);
        }
        // Longest thrust arrow spans ~8% of the trajectory, so the picture stays
        // readable whatever the vehicle's thrust happens to be.
        double span = (node[0] - node[n - 1]).Length();
        double arrow = thrustScale > 0 ? 0.08 * span / thrustScale : 0.0;

        for (int k = 0; k < n; k += 2)
        {
            double3 p = node[k];

            KsaFrameBridge.QuatToMatrix(px[k * 14 + 6], px[k * 14 + 7], px[k * 14 + 8], px[k * 14 + 9],
                                        out double3 b0, out double3 b1, out double3 b2);

            // Body +Z (the vehicle's pointing axis) in magenta — the attitude profile.
            OvLine(dl, p, p + f.VecToCci(b2) * (0.03 * span), axisCol, 1.5f);

            // Thrust vector in the site frame: R(q) * (tdx, tdy, T).
            double3 thrustSite = b0 * pu[k * 4 + 0] + b1 * pu[k * 4 + 1] + b2 * pu[k * 4 + 2];
            if (arrow > 0)
                OvLine(dl, p, p + f.VecToCci(thrustSite) * arrow, thrCol, 1.5f);
        }

        for (int k = 0; k < n; k++)
            if (TryProjectCci(node[k], out float2 s))
                dl.AddCircleFilled(s, 2.0f, nodeCol);

        // --- target and pad ---
        double3 targetCci = f.PosToCci(new double3(0, 0, _sixDofTargetAltM));
        if (TryProjectCci(targetCci, out float2 tgt))
            dl.AddCircleFilled(tgt, 5.0f, tgtCol);
        OvLine(dl, siteCci, targetCci, padCol, 1.0f);

        // --- where the tracker thinks the vehicle should be right now ---
        // Drawn against the live vehicle so plan-vs-flown divergence is visible in
        // the world, not just as a number.
        double dt = g.Sigma / (n - 1);
        double sNode = Math.Clamp(g.PlanElapsed / dt, 0.0, n - 1.001);
        int k0 = (int)sNode;
        double frac = sNode - k0;
        int k1 = Math.Min(k0 + 1, n - 1);
        double3 refCci = node[k0] * (1.0 - frac) + node[k1] * frac;
        if (TryProjectCci(refCci, out float2 rs))
        {
            dl.AddCircleFilled(rs, 4.0f, refCol);
            Vehicle v = Program.ControlledVehicle;
            if (v != null)
                OvLine(dl, refCci, v.Orbit.StateVectors.PositionCci, refCol, 1.0f);
        }

        // BeginOverlayWindow opens an ImGui window and documents that the caller
        // closes it. Omitting this produces ImGui's "missing End" assert, not a
        // silent leak.
        ImGui.End();
    }
}
