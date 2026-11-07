namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>A verb that the window brings under a resting mouse during that delay counts the same, because the height of the window follows one frame after the verb changed.</summary>
internal sealed class VerbGuard
{
    public const double ArmDelaySeconds = TakeOverAsk.ArmDelaySeconds;

    private string _verbs = "";
    private double _changedAt = double.NegativeInfinity;
    private bool _held;

    /// <summary><paramref name="moved"/> says that the verbs stand at another place than in the last frame.</summary>
    public void Update(string verbs, bool mouseOver, double now, bool moved = false)
    {
        if (verbs != _verbs)
        {
            _verbs = verbs;
            _changedAt = now;
            _held = mouseOver;
        }
        else if (!mouseOver)
            _held = false;
        else if (moved && now - _changedAt < ArmDelaySeconds)
            _held = true;
    }

    public bool MayLight => !_held;

    public bool TakesClicks(double now) => !_held || now - _changedAt >= ArmDelaySeconds;

    public void Reset()
    {
        _verbs = "";
        _changedAt = double.NegativeInfinity;
        _held = false;
    }
}
