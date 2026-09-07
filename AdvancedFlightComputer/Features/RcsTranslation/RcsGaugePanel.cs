using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

// Scale rows with the stock gauge rectangle. GaugeLabel.Pack4 requires uppercase text of at most 16 glyphs.
// Use whole values with smaller units because decimal points are not clear in the gauge atlas.
internal static class RcsGaugePanel
{
    private const int MaxChars = 16;

    // ImGauge sizes glyphs relative to their row.
    private const float TextScale = 0.34f;

    // Match the row height and pitch in BurnCanvasHost.Draw.
    #region Layout fractions

    private const float RowHeightFrac = 0.0545f;
    private const float RowGapFrac = 0.0273f;
    private const float MarginFrac = 0.025f;
    private const float LabelXFrac = 0.05f;
    private const float LabelWFrac = 0.42f;
    private const float ValueXFrac = 0.5f;
    private const float ValueWFrac = 0.45f;

    #endregion

    internal static void Draw(Burn burn, Vehicle vehicle, FlightComputer fc,
        float2 canvasMinPixels, float2 canvasSizePixels)
    {
        if (vehicle.Parts.Modules.Get<ThrusterController>().Length == 0)
            return;

        double timeSec = burn.Time.Seconds();
        double dvMs = burn.DeltaVVlf.Length();
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        RcsBurnOptions? options = exec?.FindOptions(timeSec, dvMs);
        bool isActiveBurn = exec != null && exec.IsActive
            && options != null
            && options.Matches(exec.ActiveBurnTimeSec!.Value, exec.ActiveBurnDvMs!.Value);

        RcsExecutionMode mode = options?.Mode ?? RcsExecutionMode.Default;
        RcsExecutionMode resolved = RcsExecutor.ResolveMode(vehicle, options);
        bool rcsRows = resolved == RcsExecutionMode.Rcs || isActiveBurn;

        bool noTranslation = false;
        bool showEstimates = false;
        bool currentVehicle = false;
        RcsEstimates est = default;
        bool holdEst = false;
        bool alignEst = false;
        bool shortOfPropellant = false;
        BurnTarget? bt = fc.Burn;
        if (rcsRows && !isActiveBurn)
        {
            noTranslation = !RcsExecutor.ProbeCached(vehicle).HasAnyTranslation;
            if (!noTranslation && RcsBurnPreview.TryGetEstimates(burn, vehicle, fc, exec,
                    out est, out currentVehicle))
            {
                showEstimates = true;
                holdEst = est.HoldFeasible;
                alignEst = est.AlignFeasible;
                double neededKg = est.RequiredPropellantKg(
                    options?.Attitude ?? RcsAttitudeStrategy.Auto);
                shortOfPropellant = neededKg > RcsExecutor.AvailablePropellantCached(vehicle);
            }
        }

        bool estHeader = holdEst || alignEst;

        int rows = 2;                                   // header + execution
        if (rcsRows) rows += 2;                         // attitude + allocator
        if (isActiveBurn) rows += 3;                    // status, to go, cancel
        if (noTranslation) rows += 1;
        if (estHeader) rows += 1;
        if (showEstimates && currentVehicle) rows += 1;
        if (holdEst) rows += 1;
        if (alignEst) rows += 1;
        if (shortOfPropellant) rows += 1;

        float rowH = canvasSizePixels.Y * RowHeightFrac;
        float gap = canvasSizePixels.Y * RowGapFrac;
        float margin = canvasSizePixels.Y * MarginFrac;
        float heightPixels = margin * 2f + rows * rowH + (rows - 1) * gap;

        // OffsetUv is relative to the main viewport, hence the Pos subtraction.
        float2 topLeft = canvasMinPixels + new float2(0f, canvasSizePixels.Y)
            - ImGui.GetMainViewport().Pos;
        // ImGauge's window registry has no removal API, so these resources
        // outlive a mod unload; a stable id caps that at one reused entry.
        var window = new ImGaugeWindow(
            "AfcRcsBurn", "AFC RCS Burn",
            anchorUv: float2.Zero,
            pivotUv: float2.Zero,
            offsetUv: ScreenReference.PixelsToUv(topLeft),
            sizeUv: ScreenReference.PixelsToUv(new float2(canvasSizePixels.X, heightPixels)));

        // BeginWindow always succeeds and pushes style state that EndWindow must pop.
        ImGauge.BeginWindow(in window, out float2 posPixels, out float2 sizePixels);
        try
        {
            ImGauge.DrawDressedBox(posPixels, sizePixels);

            float labelX = posPixels.X + sizePixels.X * LabelXFrac;
            float valueX = posPixels.X + sizePixels.X * ValueXFrac;
            var labelSize = new float2(sizePixels.X * LabelWFrac, rowH);
            var valueSize = new float2(sizePixels.X * ValueWFrac, rowH);
            var fullSize = new float2(sizePixels.X * (1f - LabelXFrac * 2f), rowH);
            float y = posPixels.Y + margin;

            ImGaugeStyle text = ImGaugeStyle.Default.WithText(new float3(1f, 1f, 1f), TextScale);
            ImGaugeStyle warn = ImGaugeStyle.Default.WithText(new float3(1f, 0.45f, 0.3f), TextScale);
            ImGaugeStyle dim = ImGaugeStyle.Default.WithText(new float3(0.6f, 0.62f, 0.65f), TextScale);
            // ImGauge.Button rejects clicks through its disabled style. Cancel stays enabled.
            ImGaugeStyle button = ImGaugeStyle.Default.WithText(new float3(0f, 0f, 0f), TextScale)
                .WithDisabled(isActiveBurn);
            ImGaugeStyle cancelButton = ImGaugeStyle.Default.WithText(new float3(0f, 0f, 0f), TextScale);

            void LabelRow(ReadOnlySpan<char> label, ReadOnlySpan<char> value)
            {
                ImGauge.Label(new float2(labelX, y), labelSize, label, in text);
                ImGauge.Label(new float2(valueX, y), valueSize, value, in text);
                y += rowH + gap;
            }

            bool ButtonRow(ReadOnlySpan<char> label, ReadOnlySpan<char> value, in ImGaugeStyle style)
            {
                if (!label.IsEmpty)
                    ImGauge.Label(new float2(labelX, y), labelSize, label, in text);
                // ImGauge.Button lays its hit area at the ImGui cursor; there
                // is no positioned overload.
                ImGui.SetCursorScreenPos(new float2(valueX, y));
                bool clicked = ImGauge.Button(value, valueSize, in style);
                y += rowH + gap;
                return clicked;
            }

            void FullRow(ReadOnlySpan<char> label, in ImGaugeStyle style)
            {
                ImGauge.Label(new float2(labelX, y), fullSize, label, in style);
                y += rowH + gap;
            }

            RcsBurnOptions Options() => RcsExecRegistry.GetOrCreate(vehicle.Id)
                .GetOrCreateOptions(timeSec, dvMs);

            FullRow("RCS BURN".AsSpan(), in text);

            if (ButtonRow("EXECUTION".AsSpan(), Fit(ExecutionLabel(mode, resolved)), in button))
            {
                RcsBurnOptions o = Options();
                RcsBurnUi.CycleMode(o);
            }

            if (rcsRows)
            {
                // Show the resolved strategy and allocator fallback during execution.
                RcsAttitudeStrategy attitude = options?.Attitude ?? RcsAttitudeStrategy.Auto;
                string attitudeText = isActiveBurn && exec != null
                    ? ResolvedAttitude(exec)
                    : RcsBurnUi.AttitudeLabel(attitude);
                if (ButtonRow("ATTITUDE".AsSpan(), Fit(attitudeText), in button))
                {
                    RcsBurnOptions o = Options();
                    RcsBurnUi.CycleAttitude(o);
                }

                RcsAllocator allocator = options?.Allocator ?? RcsAllocator.Groups;
                string allocatorText = isActiveBurn && exec != null
                    ? ResolvedAllocator(exec)
                    : RcsBurnUi.AllocatorLabel(allocator);
                if (ButtonRow("ALLOCATOR".AsSpan(), Fit(allocatorText), in button))
                {
                    RcsBurnOptions o = Options();
                    RcsBurnUi.CycleAllocator(o);
                }
            }

            if (isActiveBurn && exec != null)
            {
                LabelRow("STATUS".AsSpan(), Fit(StatusText(exec, fc, bt)));
                LabelRow("TO GO".AsSpan(), Fit(bt != null ? ToGo(bt.DeltaVToGoCci.Length()) : "-"));
                if (ButtonRow(ReadOnlySpan<char>.Empty, "CANCEL".AsSpan(), in cancelButton))
                {
                    RcsExecutor.RequestCancel(exec, "user request");
                }
            }

            if (noTranslation)
                FullRow("NO RCS TRANSLATE".AsSpan(), in warn);
            if (showEstimates)
            {
                if (currentVehicle)
                    FullRow("CURRENT VEHICLE".AsSpan(), in dim);
                if (estHeader)
                    FullRow("PROPELLANT/TIME".AsSpan(), in dim);
                if (holdEst)
                    LabelRow("HOLD".AsSpan(),
                        Fit(Estimate(est.HoldPropellantKg, est.HoldDurationSec)));
                if (alignEst)
                    LabelRow("ALIGN".AsSpan(),
                        Fit(Estimate(est.AlignTotalPropellantKg, est.AlignDurationSec)));
                if (shortOfPropellant)
                    FullRow("LOW PROPELLANT".AsSpan(), in warn);
            }

#if DEBUG
            // Extra rows are clipped because the rectangle is fixed before drawing.
            float drawnRows = (y - posPixels.Y - margin) / (rowH + gap);
            if (Math.Abs(drawnRows - rows) > 0.01f)
                LogHelper.WarnOnce("rcs-gauge-rows",
                    $"[AFC] RcsGaugePanel sized the window for {rows} rows " +
                    $"but drew {drawnRows:F2}.");
#endif
        }
        finally
        {
            ImGauge.EndWindow();
        }
    }

    private static string ExecutionLabel(RcsExecutionMode mode, RcsExecutionMode resolved)
        => mode == RcsExecutionMode.Default
            ? (resolved == RcsExecutionMode.Rcs ? "DEF RCS" : "DEF ENGINE")
            : RcsBurnUi.ModeLabel(mode);

    private static string ResolvedAttitude(RcsExecution exec)
        => exec.ResolvedStrategy == RcsAttitudeStrategy.Align && exec.ResolvedAxis >= 0
            ? $"ALIGN {RcsExecutor.AxisName(exec.ResolvedAxis)}"
            : RcsBurnUi.AttitudeLabel(exec.ResolvedStrategy);

    /// <summary>LP-GRP is the LP allocator running on its group fallback,
    /// which is a different state from having asked for groups.</summary>
    private static string ResolvedAllocator(RcsExecution exec)
        => exec.ResolvedAllocator == RcsAllocator.Lp
            ? (exec.LpSecondsPerImpulse != null ? "LP" : "LP-GRP")
            : "GROUPS";

    private static string StatusText(RcsExecution exec, FlightComputer fc, BurnTarget? bt)
    {
        if (bt == null)
            return "WAITING";
        double toIgnition = (bt.IgnitionTime - Universe.GetElapsedTime()).Seconds();
        if (toIgnition > 0.0)
            return $"T-{toIgnition:F0} S";
        return exec.ResolvedStrategy == RcsAttitudeStrategy.Align && RcsExecutor.OutsideAlignGate(fc)
            ? "ALIGNING"
            : "FIRING";
    }

    /// <summary>Switches to cm/s below 10 m/s so the end of a trim burn still
    /// shows progress instead of collapsing to "0 M/S".</summary>
    private static string ToGo(double ms)
        => ms >= 10.0 ? $"{ms:F0} M/S" : $"{ms * 100.0:F0} CM/S";

    /// <summary>Propellant and duration in one cell, e.g. "907 KG/10 S";
    /// worst case "9999 KG/89 MIN" is 14 glyphs.</summary>
    private static string Estimate(double kg, double sec)
        => $"{Mass(kg)}/{Duration(sec)}";

    private static string Mass(double kg)
        => kg >= 10000.0 ? $"{kg / 1000.0:F0} T" : $"{kg:F0} KG";

    private static string Duration(double sec)
        => sec >= 5400.0 ? $"{sec / 3600.0:F0} HR"
            : sec >= 90.0 ? $"{sec / 60.0:F0} MIN"
            : $"{sec:F0} S";

    private static ReadOnlySpan<char> Fit(string text)
    {
        // Normal labels are already uppercase. Keep unusual numeric text readable in the gauge atlas too.
        foreach (char c in text)
        {
            if (!char.IsLower(c))
                continue;
            text = text.ToUpperInvariant();
            break;
        }
        return text.AsSpan(0, Math.Min(text.Length, MaxChars));
    }
}
