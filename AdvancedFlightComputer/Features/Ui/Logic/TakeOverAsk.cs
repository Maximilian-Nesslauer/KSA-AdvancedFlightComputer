using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class TakeOverAsk
{
    public const double ArmDelaySeconds = 0.4;

    private sealed record Pending(string Verb, string From, string Message, Action Confirm, DateTime Raised);

    private static Pending? _pending;

    public static bool Active => _pending != null;

    public static void Raise(string verb, string from, string message, Action confirm, DateTime now)
        => _pending = new Pending(verb, from, message, confirm, now);

    public static void Clear() => _pending = null;

    /// <summary>The commit to show this frame: the commit of the page, or while the question stands its CONFIRM and CANCEL form.</summary>
    public static CommitSpec Apply(CommitSpec commit, DateTime now)
    {
        Pending? ask = _pending;
        if (ask == null)
            return commit;
        if (!commit.NeedsTakeOver || commit.Verb != ask.Verb || commit.TakeOverFrom != ask.From)
        {
            _pending = null;
            return commit;
        }

        bool armed = (now - ask.Raised).TotalSeconds >= ArmDelaySeconds;
        return commit with
        {
            Verb = "CONFIRM: " + (commit.EndsLabel.Length > 0 ? commit.EndsLabel : "TAKE OVER"),
            EndsLabel = "",
            NeedsTakeOver = false,
            Status = ask.Message,
            StatusPending = false,
            Caution = "",
            Tooltip = "",
            Outcome = "",
            Act = armed
                ? () =>
                {
                    _pending = null;
                    ask.Confirm();
                }
                : null,
            Secondary = new CommitAction("CANCEL", "", "Keeps what flies now.", Clear, Danger: false),
        };
    }
}
