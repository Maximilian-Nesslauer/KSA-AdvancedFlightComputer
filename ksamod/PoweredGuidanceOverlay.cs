using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using Gfold;
using KSA;

// World-space debug overlay for the G-FOLD descent. Projects the committed plan
// and its constraints into the live game view so you can see exactly what the
// solver is working with: the planned path, the glideslope cone, the target, the
// commanded thrust at each node, the live vehicle state, and a numeric HUD.
//
// Drawn on its own transparent, click-through, full-screen ImGui window using the
// active camera's world->screen projection. There is no depth test, so lines draw
// on top of terrain rather than being occluded by it — fine for a debug view.
public static partial class PoweredGuidanceWindow
{
    private static bool _showGfoldOverlay;
    private static bool _retargetArmed;
    private static bool _landingTabActive;   // set while the Landing tab is the open tab

    // Clickable retargeting: while armed, each frame we ray-cast the cursor onto the
    // body, draw a live preview marker (projected back through the validated forward
    // EclToScreen, so it should sit under the cursor), and commit the new site on a
    // left-click. Right-click cancels.
    private static void HandleRetargetClick(Viewport vp, IParentBody parent)
    {
        if (!_retargetArmed)
            return;
        if (ImGui.IsMouseClicked(ImGuiMouseButton.Right))
        {
            _retargetArmed = false;
            _landingStatus = "Retarget cancelled.";
            return;
        }
        Camera cam = Program.GetMainCamera();
        if (cam == null)
            return;

        bool overUi = ImGui.GetIO().WantCaptureMouse;
        double3 hitEcl = default;
        double latDeg = 0, lonDeg = 0;
        bool hit = !overUi && RaycastSurface(cam, vp, parent, out hitEcl, out latDeg, out lonDeg);
        DrawRetargetPreview(vp, cam, hit, hitEcl, latDeg, lonDeg);

        if (hit && ImGui.IsMouseClicked(ImGuiMouseButton.Left))
        {
            RetargetGfold(latDeg, lonDeg);
            _retargetArmed = false;
        }
    }

    // Cast a ray from the camera through the cursor and intersect the body sphere.
    // Returns the hit (ECL) and its lat/lon (CCF), or false if the cursor misses.
    private static bool RaycastSurface(Camera cam, Viewport vp, IParentBody parent,
                                       out double3 hitEcl, out double latDeg, out double lonDeg)
    {
        hitEcl = default;
        latDeg = 0;
        lonDeg = 0;
        if (parent is not Celestial body)
            return false;

        // Cursor pixel -> NDC. Normalize by ImGui's display size (the space the mouse
        // is in). Vulkan NDC has y pointing DOWN, so screen y maps straight through
        // (no flip).
        float2 mp = ImGui.GetMousePos();
        float2 disp = ImGui.GetIO().DisplaySize;
        if (disp.X < 1f || disp.Y < 1f)
            return false;
        double nx = 2.0 * mp.X / disp.X - 1.0;
        double ny = 2.0 * mp.Y / disp.Y - 1.0;

        // Ray through the cursor. Vulkan reverse-Z: the near plane is z=1, far is z=0,
        // so the origin (camera side) is the z=1 unprojection and the ray runs toward
        // z=0. Getting this backwards makes the near sphere root land on the BACK of
        // the body (which still projects under the cursor, hence the camera-dependent,
        // antipodal lat/lon).
        double3 origin = cam.EgoToEcl(cam.NdcToEgo(new double3(nx, ny, 1.0)));
        double3 farPt = cam.EgoToEcl(cam.NdcToEgo(new double3(nx, ny, 0.0)));
        double3 dir = double3.Normalize(farPt - origin);

        double3 center = body.GetPositionEcl();
        doubleQuat ecl2ccf = doubleQuat.Inverse(body.GetBodyFixed2Ecl());

        // First pass against the mean-radius sphere.
        if (!IntersectSphere(origin, dir, center, parent.MeanRadius, out double t))
            return false;
        // Hit direction from the body centre -> CCF -> lat/lon (inverse of SiteDirCcf:
        // lat = asin(z), lon = atan2(y, x)).
        double3 ccf = double3.Normalize(origin + dir * t - center).Transform(ecl2ccf);

        // Refine once to the terrain height there: the visible surface sits at
        // MeanRadius + terrain, so the mean sphere reads slightly off (worse at grazing
        // angles) — this pulls the hit onto the surface actually under the cursor.
        double terrain = body.GetTerrainHeightFromDirCcf(ccf);
        if (double.IsFinite(terrain) &&
            IntersectSphere(origin, dir, center, parent.MeanRadius + terrain, out double t2))
        {
            t = t2;
            ccf = double3.Normalize(origin + dir * t - center).Transform(ecl2ccf);
        }

        hitEcl = origin + dir * t;
        latDeg = Math.Asin(Math.Clamp(ccf.Z, -1.0, 1.0)) * 180.0 / Math.PI;
        lonDeg = Math.Atan2(ccf.Y, ccf.X) * 180.0 / Math.PI;
        return true;
    }

    // Nearest forward intersection of a ray with a sphere; false on a miss or if both
    // roots are behind the origin.
    private static bool IntersectSphere(double3 origin, double3 dir, double3 center, double radius, out double t)
    {
        t = 0.0;
        double3 oc = origin - center;
        double b = double3.Dot(oc, dir);
        double c = double3.Dot(oc, oc) - radius * radius;
        double disc = b * b - c;
        if (disc < 0.0)
            return false;
        double sd = Math.Sqrt(disc);
        t = -b - sd;
        if (t < 0.0) t = -b + sd;
        return t >= 0.0;
    }

    // Live preview while armed: a marker at the projected hit (should track the cursor
    // if the inverse projection is correct) plus the lat/lon it would set.
    private static void DrawRetargetPreview(Viewport vp, Camera cam, bool hit,
                                            double3 hitEcl, double latDeg, double lonDeg)
    {
        const ImGuiWindowFlags flags =
            ImGuiWindowFlags.NoDecoration | ImGuiWindowFlags.NoMove |
            ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoBringToFrontOnFocus |
            ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoInputs |
            ImGuiWindowFlags.NoBackground;
        ImGui.SetNextWindowPos(new float2(0f, 0f));
        ImGui.SetNextWindowSize(new float2(vp.Width, vp.Height));
        ImGui.Begin("##retarget_preview", flags);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float2 m = ImGui.GetMousePos();
        var col = new ImColor8(255, 90, 220);
        if (hit)
        {
            float2 s = cam.EclToScreen(hitEcl, false);
            dl.AddCircleFilled(s, 6f, col);
            dl.AddText(s + new float2(10f, 6f), col, $"retarget  lat {latDeg:F2}  lon {lonDeg:F2}");
            dl.AddText(m + new float2(12f, -16f), new ImColor8(150, 150, 150), "cursor");
        }
        else
        {
            dl.AddText(m + new float2(12f, 6f), new ImColor8(200, 200, 200), "aim at the surface to retarget");
        }
        ImGui.End();
    }

    // Move the landing site, and (if a descent is live) force a fresh G-FOLD search to
    // the new spot on the next step rather than reusing the old arrival time.
    private static void RetargetGfold(double latDeg, double lonDeg)
    {
        _siteLatDeg = latDeg;
        _siteLonDeg = lonDeg;
        _landingStatus = $"Retargeted to lat {latDeg:F3}, lon {lonDeg:F3}.";
        if (_landingPhase == LandingPhase.GfoldDescent || _landingPhase == LandingPhase.VerticalDescent)
        {
            _gfoldForceSearch = true;
            _gfoldLastSolveTime = double.NegativeInfinity;
            _gfoldArrivalTime = SimNow() + 120.0; // out of the terminal-freeze window so it re-solves
            _gfoldFailStreak = 0;
        }
    }

    // Per-frame projection context, set at the top of DrawGfoldOverlay so the
    // helpers don't each re-fetch the camera and body transforms.
    private static Camera _ovCam;
    private static double3 _ovBodyEcl;     // landing body's position in ECL
    private static doubleQuat _ovCci2Ccf;  // body inertial -> body fixed
    private static doubleQuat _ovCcf2Ecl;  // body fixed -> ECL
    private static readonly float2[] _ovSeg = new float2[2];

    private static void DrawGfoldOverlay(Viewport vp, Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (!_showGfoldOverlay)
            return;

        GfoldTrajectory plan = _gfoldPlan;
        bool active = _landingPhase == LandingPhase.GfoldDescent
                   || _landingPhase == LandingPhase.VerticalDescent;
        if (plan == null || !active)
            return;
        if (!SetupProjection(parent))
            return;

        // Rebuild the site frame live (the plan is flown in the body-fixed, rotating
        // pad frame, not the solve-time one), so the overlay sits on the real ground
        // instead of drifting off it as the body turns.
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaGfold.Frame f = KsaGfold.BuildFrame(siteCci);
        int n = plan.Nodes;
        double now = SimNow();

        var trajCol = new ImColor8(70, 220, 100);
        var nodeCol = new ImColor8(150, 255, 170);
        var coneCol = new ImColor8(235, 150, 40);
        var tgtCol = new ImColor8(255, 90, 220);
        var padCol = new ImColor8(235, 235, 235);
        var thrCol = new ImColor8(255, 215, 60);
        var liveCol = new ImColor8(80, 220, 255);
        var velCol = new ImColor8(150, 240, 255);
        var devCol = new ImColor8(255, 80, 80);
        var hudCol = new ImColor8(205, 215, 225);

        const ImGuiWindowFlags flags =
            ImGuiWindowFlags.NoDecoration | ImGuiWindowFlags.NoMove |
            ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoBringToFrontOnFocus |
            ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoInputs |
            ImGuiWindowFlags.NoBackground;
        ImGui.SetNextWindowPos(new float2(0f, 0f));
        ImGui.SetNextWindowSize(new float2(vp.Width, vp.Height));
        ImGui.Begin("##gfold_overlay", flags);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();

        // --- Glideslope cone (drawn first so the path sits on top of it). The
        // constraint is ||r_horizontal|| <= cot(gs) * height-above-target, i.e. a
        // cone with apex at the target opening upward; rings + a few ribs show it.
        double tx = plan.Position[n - 1][0];                       // target altitude (local up)
        double topAlt = Math.Max(plan.Position[0][0], _gfoldAltM); // draw up to the start/current
        double cot = 1.0 / Math.Tan(Math.Max(_gfoldGlideSlopeDeg, 1.0) * Math.PI / 180.0);
        double3 apex = PlanCci(f, new double3(tx, 0, 0));
        const int rings = 4, seg = 28;
        for (int k = 1; k <= rings; k++)
        {
            double dx = (topAlt - tx) * k / rings;
            if (dx <= 0) continue;
            double rad = cot * dx, bx = tx + dx;
            double3 prev = default;
            for (int j = 0; j <= seg; j++)
            {
                double th = 2.0 * Math.PI * j / seg;
                double3 p = PlanCci(f, new double3(bx, rad * Math.Cos(th), rad * Math.Sin(th)));
                if (j > 0) OvLine(dl, prev, p, coneCol, 1.3f);
                prev = p;
            }
        }
        double topDx = topAlt - tx, topRad = cot * topDx;
        for (int a = 0; a < 4; a++)
        {
            double th = Math.PI / 2.0 * a;
            double3 rim = PlanCci(f, new double3(tx + topDx, topRad * Math.Cos(th), topRad * Math.Sin(th)));
            OvLine(dl, apex, rim, coneCol, 1.3f);
        }

        // --- Commanded thrust at each node (short ray along AccelCmd) ---
        const double thrustScale = 4.0; // metres drawn per m/s^2
        int tstep = Math.Max(1, n / 16);
        for (int i = 0; i < n; i += tstep)
        {
            double3 baseL = Node(plan.Position, i);
            double3 tipL = baseL + Node(plan.AccelCmd, i) * thrustScale;
            OvLine(dl, PlanCci(f, baseL), PlanCci(f, tipL), thrCol, 2.0f);
        }

        // --- Planned trajectory polyline ---
        for (int i = 0; i < n - 1; i++)
            OvLine(dl, PlanCci(f, Node(plan.Position, i)), PlanCci(f, Node(plan.Position, i + 1)), trajCol, 2.5f);

        // --- Node dots + altitude labels ---
        int nstep = Math.Max(1, n / 12);
        for (int i = 0; i < n; i += nstep)
            if (TryProjectCci(PlanCci(f, Node(plan.Position, i)), out float2 p))
            {
                dl.AddCircleFilled(p, 3f, nodeCol);
                dl.AddText(p + new float2(5f, -6f), nodeCol, $"{plan.Position[i][0]:F0}m");
            }

        // --- Target + pad markers ---
        if (TryProjectCci(PlanCci(f, Node(plan.Position, n - 1)), out float2 tgt))
        {
            dl.AddCircleFilled(tgt, 6f, tgtCol);
            dl.AddText(tgt + new float2(8f, -6f), tgtCol, "TARGET");
        }
        if (TryProjectCci(f.Origin, out float2 pad))
        {
            dl.AddCircleFilled(pad, 5f, padCol);
            dl.AddText(pad + new float2(8f, -6f), padCol, "PAD");
        }

        // --- Live vehicle state: CoM (the flown reference), velocity, plan deviation ---
        double3 com = orbit.StateVectors.PositionCci;
        double3 vSrf = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), com);
        OvLine(dl, com, com + vSrf * 1.5, velCol, 2.0f); // velocity vector (~1.5 s lookahead)
        if (TryProjectCci(com, out float2 lp))
        {
            dl.AddCircleFilled(lp, 5f, liveCol);
            dl.AddText(lp + new float2(8f, -6f), liveCol, "CG");
        }

        // Deviation: current CoM vs. where the plan says it should be now.
        double elapsed = now - _gfoldPlanStart;
        double sf = Math.Clamp(elapsed / plan.Dt, 0.0, n - 1);
        int s0 = Math.Clamp((int)Math.Floor(sf), 0, n - 2);
        double sfrac = Math.Clamp(sf - s0, 0.0, 1.0);
        double3 refLocal = Lerp(Node(plan.Position, s0), Node(plan.Position, s0 + 1), sfrac);
        OvLine(dl, com, PlanCci(f, refLocal), devCol, 1.5f);
        double devM = (refLocal - f.PointToLocal(com)).Length();

        // --- Numeric HUD (top-right) ---
        double tgo = _gfoldArrivalTime - now;
        string[] hud =
        {
            $"G-FOLD   {_gfoldStatus}",
            $"phase    {_landingPhase}",
            $"tgo     {tgo,6:F1} s   tf {plan.TimeOfFlight,5:F1} s",
            $"alt     {_gfoldAltM,7:F0} m",
            $"speed   {_gfoldSpeedMs,6:F1} / {_gfoldVMaxMs,5:F0} m/s",
            $"throttle{_gfoldThrottle * 100,5:F0} %",
            $"fuel    {vehicle.PropellantMass,7:F0} kg",
            $"deviation{devM,6:F1} m",
            $"land err{plan.LandingErrorNorm,6:F1} m",
            $"nodes {plan.Nodes}  iters {plan.Iterations}",
        };
        float hx = vp.Width - 300f, hy = 70f, lh = 16f;
        dl.AddRectFilled(new float2(hx - 8f, hy - 6f),
            new float2(hx + 292f, hy + hud.Length * lh + 6f), new ImColor8(10, 14, 20), 4f);
        for (int i = 0; i < hud.Length; i++)
            dl.AddText(new float2(hx, hy + i * lh), hudCol, hud[i]);

        ImGui.End();
    }

    // Set the per-frame projection context (camera + body transforms) used by
    // TryProjectCci. False if there's no camera or the parent isn't a Celestial.
    private static bool SetupProjection(IParentBody parent)
    {
        if (parent is not Celestial body)
            return false;
        Camera cam = Program.GetMainCamera();
        if (cam == null)
            return false;
        _ovCam = cam;
        _ovBodyEcl = body.GetPositionEcl();
        _ovCci2Ccf = parent.GetCci2Ccf();
        _ovCcf2Ecl = body.GetBodyFixed2Ecl();
        return true;
    }

    // The landing-site marker, drawn in the world whenever the Landing tab is open
    // (independent of G-FOLD), so the target is visible for deorbit/UPFG planning too.
    private static void DrawLandingSiteMarker(Viewport vp, IParentBody parent)
    {
        if (!SetupProjection(parent))
            return;
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        if (!TryProjectCci(siteCci, out float2 s))
            return;

        const ImGuiWindowFlags flags =
            ImGuiWindowFlags.NoDecoration | ImGuiWindowFlags.NoMove |
            ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoBringToFrontOnFocus |
            ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoInputs |
            ImGuiWindowFlags.NoBackground;
        ImGui.SetNextWindowPos(new float2(0f, 0f));
        ImGui.SetNextWindowSize(new float2(vp.Width, vp.Height));
        ImGui.Begin("##landing_site", flags);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        var col = new ImColor8(120, 230, 255);
        const float g = 5f, r = 13f;            // crosshair gap and reach
        ScreenLine(dl, s + new float2(g, 0f), s + new float2(r, 0f), col, 1.6f);
        ScreenLine(dl, s - new float2(r, 0f), s - new float2(g, 0f), col, 1.6f);
        ScreenLine(dl, s + new float2(0f, g), s + new float2(0f, r), col, 1.6f);
        ScreenLine(dl, s - new float2(0f, r), s - new float2(0f, g), col, 1.6f);
        dl.AddCircleFilled(s, 2.5f, col);
        dl.AddText(s + new float2(r + 4f, -6f), col, $"SITE  {_siteLatDeg:F3}, {_siteLonDeg:F3}");
        ImGui.End();
    }

    private static void ScreenLine(ImDrawListPtr dl, float2 a, float2 b, ImColor8 col, float thick)
    {
        _ovSeg[0] = a;
        _ovSeg[1] = b;
        dl.AddPolyline(_ovSeg, col, ImDrawFlags.None, thick);
    }

    // A point in the plan's site-local frame (x = up) back to a CCI position.
    private static double3 PlanCci(KsaGfold.Frame f, double3 local) => f.Origin + f.VecToCci(local);

    // Project a CCI point to screen pixels; false if it is behind the camera. The
    // chain is CCI -> body-fixed -> ECL (+ body ECL position) -> screen.
    private static bool TryProjectCci(double3 cci, out float2 screen)
    {
        double3 ecl = _ovBodyEcl + cci.Transform(_ovCci2Ccf).Transform(_ovCcf2Ecl);
        float4 clip = _ovCam.EgoToClip(_ovCam.EclToEgo(ecl));
        if (clip.W <= 0.001f) { screen = default; return false; } // behind the camera
        screen = _ovCam.EclToScreen(ecl, false);
        return true;
    }

    private static void OvLine(ImDrawListPtr dl, double3 cciA, double3 cciB, ImColor8 col, float thick)
    {
        if (TryProjectCci(cciA, out float2 a) && TryProjectCci(cciB, out float2 b))
        {
            _ovSeg[0] = a;
            _ovSeg[1] = b;
            dl.AddPolyline(_ovSeg, col, ImDrawFlags.None, thick);
        }
    }
}
