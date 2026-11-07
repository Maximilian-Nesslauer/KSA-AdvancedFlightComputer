#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

/// <summary>
/// The question of a reset. It is the stock ConfirmActionPopup with a verb that says what the press does and the height of its text, because the stock popup holds the size of a long form and answers with CONFIRM.
/// </summary>
internal sealed class ResetPopup : SuccessActionPopup
{
    private static readonly PopupButton<Popup>[] Buttons =
    {
        CreateButton<Popup>(PageText.ResetVerb, static popup =>
        {
            (popup as ResetPopup)?.OnSuccess();
            popup.Active = false;
        }),
        PopupButtonCancel,
    };

    private readonly string _title;
    private readonly string _message;

    private ResetPopup(Action onSuccess, string title, string message)
        : base(onSuccess)
    {
        _title = title.ToUpperInvariant();
        _message = message;
    }

    /// <summary>Raises the question. The popup list holds it from here on, as for every stock popup.</summary>
    public static ResetPopup Create(Action action, string title, string message) => new(action, title, message);

    protected override void OnDrawUi()
    {
        // The stock modal grows to the body it measured in the frame before, so the height here is only the start.
        float2 size = ScreenReference.UvToPixels(DEFAULT_SIZE_UV) * GameSettings.GetInterfaceScale();
        if (!BeginConsoleModalSized(WindowId.AsSpan(), _title.AsSpan(), new float2(size.X, ConsoleStyle.TitlebarHeight + ConsoleStyle.FooterHeight)))
            return;
        ConsoleStyle.BeginBody();
        ConsoleStyle.PushWidgetStyle();
        CenteredWrappedText(_message.AsSpan());
        EndConsoleBody();
        DrawConsoleButtonRow(this, Buttons);
        EndConsoleModal();
    }
}
