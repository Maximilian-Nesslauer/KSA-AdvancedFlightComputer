using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static class RcsBurnUi
{
    internal static void DrawHelperLine(Burn burn, Vehicle vehicle)
    {
        if (burn.ParentDepartureBurn)
            return;
        if (vehicle.Parts.Modules.Get<ThrusterController>().Length == 0)
            return;

        double timeSec = burn.Time.Seconds();
        double dvMs = burn.DeltaVVlf.Length();
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        RcsBurnOptions? options = exec?.FindOptions(timeSec, dvMs);
        bool isActiveBurn = exec != null && exec.IsActive
            && options != null
            && options.Matches(exec.ActiveBurnTimeSec!.Value, exec.ActiveBurnDvMs!.Value);
        RcsExecutionMode resolved = RcsExecutor.ResolveMode(vehicle, options);

        ImGui.Separator();
        string state = exec?.Faulted == true ? "RCS STOPPED, SEE AFC > BURNS"
            : isActiveBurn ? "AFC FLIES THIS BURN WITH RCS"
            : resolved == RcsExecutionMode.Rcs ? "AFC RCS ON STOCK AUTO"
            : "STOCK AUTO, THE ENGINE";
        ConsoleWidgets.Readout("BURN WITH".AsSpan(), state.AsSpan());
        if (ConsoleWidgets.Button("OPEN AFC".AsSpan(), "AfcRcsHelperOpen".AsSpan(), default))
            Ui.Console.Pages.RcsPage.OpenFor(burn);
        if (ImGui.IsItemHovered())
            ConsoleUi.Tooltip("RCS options, estimates and the stop are on the BURNS page of the AFC console.".AsSpan());
    }

    internal static bool HasEstimatesFor(Burn burn, BurnTarget? loaded, RcsExecution? exec)
        // FlightComputer loads only the first executable burn. A later editor must not show its estimates.
        => exec != null && exec.Estimates.Valid && StockBurnIdentity.IsLoaded(loaded, burn);

    internal static void CycleMode(RcsBurnOptions options)
        => options.Mode = options.Mode == RcsExecutionMode.Rcs ? RcsExecutionMode.Default : RcsExecutionMode.Rcs;

    // GaugeLabel.Pack4 has no comma, so the label leaves it out.
    internal static string ModeLabel(RcsExecutionMode mode) => mode switch
    {
        RcsExecutionMode.Default => "ENGINE ELSE RCS",
        RcsExecutionMode.Engine => "ENGINE",
        RcsExecutionMode.Rcs => PageText.FlyWithLabels[1],
        _ => mode.ToString(),
    };
}
