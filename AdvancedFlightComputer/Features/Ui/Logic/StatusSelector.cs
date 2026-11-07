using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum Severity { Diag = -1, Info = 0, Cue, Warning, Error }

internal readonly record struct StatusLine(Severity Severity, string Text, string CueTarget, string CueLabel = "");

internal enum HeadRowHolds { Situation, Failure, Message }

internal static class StatusSelector
{
    /// <summary>A failed guidance step during staging is normal, so the head shows the step error only after this many failed steps in a row.</summary>
    public const int SustainedFailSteps = 60;

    public const double NoPlanWaitSeconds = 10.0;

    public const double NoPlanWaitRealSeconds = 5.0;

    public const double EndedNoteSeconds = 15.0;

    public const double EndedWarningSeconds = 30.0;

    /// <summary>It arrives in the first frame of the mode that commit started, where a plain note takes no row (<see cref="PickRow"/>), and it says that a craft switch changed.</summary>
    public const Severity CommitNote = Severity.Warning;

    public const double SameEventSeconds = 2.0;

    public static bool SameEvent(Severity earlier, Severity later, bool sameCraft, double secondsApart)
        => sameCraft && earlier == Severity.Info && later == Severity.Info && secondsApart >= 0.0 && secondsApart <= SameEventSeconds;

    public static bool GuidanceFaultSustained(int failStreak) => failStreak >= SustainedFailSteps;

    public static bool StepFaultLasts(bool stepRuns, string error, int failStreak)
        => stepRuns && error.Length > 0 && GuidanceFaultSustained(failStreak);

    public static bool NoPlanSustained(double simSeconds, double realSeconds)
        => simSeconds >= NoPlanWaitSeconds && realSeconds >= NoPlanWaitRealSeconds;

    public static int Pick(IReadOnlyList<StatusLine> lines, string except = "")
    {
        int best = -1;
        for (int i = 0; i < lines.Count; i++)
        {
            if (string.IsNullOrEmpty(lines[i].Text) || lines[i].Text == except)
                continue;
            if (best < 0 || lines[i].Severity > lines[best].Severity)
                best = i;
        }
        return best;
    }

    public static int PickRow(IReadOnlyList<StatusLine> lines, bool flying, string except = "")
    {
        int best = Pick(lines, except);
        return flying && best >= 0 && lines[best].Severity == Severity.Info ? -1 : best;
    }

    public static HeadRowHolds IdleRow(bool failure, bool falling, bool message)
        => failure ? HeadRowHolds.Failure
            : falling || !message ? HeadRowHolds.Situation
            : HeadRowHolds.Message;

    public static int MoreCount(IReadOnlyList<StatusLine> lines, bool rowShown = true)
    {
        int count = 0;
        for (int i = 0; i < lines.Count; i++)
            if (!string.IsNullOrEmpty(lines[i].Text))
                count++;
        return Math.Max(0, rowShown ? count - 1 : count);
    }

    public static bool ShowsInHead(Severity severity, bool modeLive, double ageSeconds, bool failureAcknowledged) => severity switch
    {
        Severity.Diag => false,
        Severity.Cue => true,
        Severity.Error => modeLive || !failureAcknowledged,
        Severity.Warning => modeLive || ageSeconds < EndedWarningSeconds,
        _ => modeLive || ageSeconds < EndedNoteSeconds,
    };
}

/// <summary>A text counts from the frame it first appears, until it is gone from the list of the raw lines.</summary>
internal sealed class LineAges
{
    private readonly Dictionary<string, double> _firstSeen = new(StringComparer.Ordinal);
    private readonly HashSet<string> _present = new(StringComparer.Ordinal);

    public void Filter(List<StatusLine> lines, bool modeLive, IReadOnlySet<string> acknowledgedLines, double nowSeconds)
    {
        _present.Clear();
        for (int i = lines.Count - 1; i >= 0; i--)
        {
            StatusLine line = lines[i];
            if (string.IsNullOrEmpty(line.Text))
                continue;
            _present.Add(line.Text);
            if (!_firstSeen.TryGetValue(line.Text, out double first))
                _firstSeen[line.Text] = first = nowSeconds;
            if (!StatusSelector.ShowsInHead(line.Severity, modeLive, nowSeconds - first, acknowledgedLines.Contains(line.Text)))
                lines.RemoveAt(i);
        }

        // Dictionary.Remove does not invalidate a running enumerator of the dictionary.
        foreach (string text in _firstSeen.Keys)
        {
            if (!_present.Contains(text))
                _firstSeen.Remove(text);
        }
    }

    public void Clear()
    {
        _firstSeen.Clear();
        _present.Clear();
    }
}
