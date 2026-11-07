using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.Features.Ui.Logic;
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
        bool faulted = exec?.Faulted == true;
        bool cleanupPending = exec?.CleanupPending == true;
        BurnTarget? bt = fc.Burn;

        // The warnings of the burn this block shows, which need not be the next node.
        bool noTranslation = false;
        bool shortOfPropellant = false;
        if (resolved == RcsExecutionMode.Rcs && !isActiveBurn && !faulted)
        {
            noTranslation = !RcsExecutor.ProbeCached(vehicle).HasAnyTranslation;
            if (!noTranslation && RcsBurnPreview.TryGetEstimates(burn, vehicle, fc, exec, out RcsEstimates est, out _))
            {
                double neededKg = est.RequiredPropellantKg(options?.Attitude ?? RcsAttitudeStrategy.Auto);
                shortOfPropellant = neededKg > RcsExecutor.AvailablePropellantCached(vehicle);
            }
        }

        int rows = 3;                                   // header, BURN WITH, OPEN AFC
        if (isActiveBurn) rows += 3;                    // status, to go, cancel
        if (faulted) rows += cleanupPending ? 2 : 1;
        if (noTranslation) rows += 1;
        if (shortOfPropellant) rows += 1;

        float rowH = canvasSizePixels.Y * RowHeightFrac;
        float gap = canvasSizePixels.Y * RowGapFrac;
        float margin = canvasSizePixels.Y * MarginFrac;
        float heightPixels = margin * 2f + rows * rowH + (rows - 1) * gap;

        // OffsetUv is relative to the main viewport, hence the Pos subtraction.
        float2 topLeft = canvasMinPixels + new float2(0f, canvasSizePixels.Y)
            - ImGui.GetMainViewport().Pos;
        // ImGauge's window registry has no removal API, so these resources outlive a mod unload; a stable id caps that at one reused entry.
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
            // ImGauge.Button rejects clicks through its disabled style. CANCEL and OPEN AFC stay enabled.
            ImGaugeStyle button = ImGaugeStyle.Default.WithText(new float3(0f, 0f, 0f), TextScale)
                .WithDisabled(isActiveBurn || cleanupPending);
            ImGaugeStyle plainButton = ImGaugeStyle.Default.WithText(new float3(0f, 0f, 0f), TextScale);

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
                // ImGauge.Button lays its hit area at the ImGui cursor; there is no positioned overload.
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

            FullRow("RCS BURN".AsSpan(), in text);

            if (faulted)
            {
                FullRow(cleanupPending ? "CLEANUP FAILED".AsSpan() : "RCS FAULT STOP".AsSpan(), in warn);
                if (cleanupPending && ButtonRow(ReadOnlySpan<char>.Empty, "RETRY CLEANUP".AsSpan(), in plainButton))
                    RcsStop.RetryCleanup(exec!);
            }

            if (ButtonRow("BURN WITH".AsSpan(), Fit(ExecutionLabel(mode, resolved)), in button))
            {
                RcsBurnOptions o = RcsExecRegistry.GetOrCreate(vehicle.Id).GetOrCreateOptions(timeSec, dvMs);
                RcsBurnUi.CycleMode(o);
            }

            if (isActiveBurn && exec != null)
            {
                LabelRow("STATUS".AsSpan(), Fit(StatusText(exec, fc, bt)));
                LabelRow("TO GO".AsSpan(), Fit(bt != null ? ToGo(bt.DeltaVToGoCci.Length()) : "-"));
                // The one RCS cancel path: an explicit stop does not give stock AUTO back.
                if (ButtonRow(ReadOnlySpan<char>.Empty, "CANCEL".AsSpan(), in plainButton))
                    RcsStop.Cancel(exec);
            }

            if (noTranslation)
                FullRow("NO RCS TRANSLATE".AsSpan(), in warn);
            if (shortOfPropellant)
                FullRow("LOW PROPELLANT".AsSpan(), in warn);

            if (ButtonRow(ReadOnlySpan<char>.Empty, "OPEN AFC".AsSpan(), in plainButton))
                Ui.Console.Pages.RcsPage.OpenFor(burn);

#if DEBUG
            // Extra rows are clipped because the rectangle is fixed before drawing.
            float drawnRows = (y - posPixels.Y - margin) / (rowH + gap);
            if (Math.Abs(drawnRows - rows) > 0.01f)
                LogHelper.WarnOnce("rcs-gauge-rows",
                    $"[AFC] RcsGaugePanel sized the window for {rows} rows but drew {drawnRows:F2}.");
#endif
        }
        finally
        {
            ImGauge.EndWindow();
        }
    }

    // GaugeLabel.Pack4 draws only A-Z, 0-9, space, '_', '-', '.', '/', '\' and '+', so these labels leave out the comma and the brackets.
    private static string ExecutionLabel(RcsExecutionMode mode, RcsExecutionMode resolved)
        => mode == RcsExecutionMode.Default
            ? (resolved == RcsExecutionMode.Rcs ? "RCS - NO ENGINE" : "ENGINE ELSE RCS")
            : RcsBurnUi.ModeLabel(mode);

    /// <summary>LP-GRP is the LP allocator running on its group fallback, which is a different state from having asked for groups.</summary>
    internal static string ResolvedAllocator(RcsExecution exec)
        => exec.ResolvedAllocator == RcsAllocator.Lp
            ? (exec.LpSecondsPerImpulse != null ? "LEAST FUEL" : "LEAST FUEL, BY AXIS")
            : "BY AXIS";

    private static string StatusText(RcsExecution exec, FlightComputer fc, BurnTarget? bt)
    {
        if (bt == null)
            return "WAITING";
        double toIgnition = (bt.IgnitionTime - Universe.GetElapsedTime()).Seconds();
        if (toIgnition > 0.0)
            return FormattableString.Invariant($"T-{toIgnition:F0} S");
        return exec.ResolvedStrategy == RcsAttitudeStrategy.Align && RcsExecutor.OutsideAlignGate(fc)
            ? "ALIGNING"
            : "FIRING";
    }

    /// <summary>Switches to cm/s below 10 m/s so the end of a trim burn still
    /// shows progress instead of collapsing to "0 M/S".</summary>
    private static string ToGo(double ms)
        => ms >= 10.0 ? FormattableString.Invariant($"{ms:F0} M/S") : FormattableString.Invariant($"{ms * 100.0:F0} CM/S");

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
