using System.Runtime.CompilerServices;
using System.Text.RegularExpressions;

// ImGui.TextColored, TextDisabled, TextWrapped, LabelText, BulletText and SetTooltip of the binding, and the
// stock ConsoleWidgets readouts and tooltips built on them, read their text as a printf format, so a '%'
// in a dynamic text such as "at 3 % throttle" makes the native side fail fast and ends the game. This scan
// finds every such call in the shared console code and the constellation feature whose text is neither a
// literal without '%' nor passed through ConsoleUi.EscapeFormat.
internal static partial class PrintfTextCalls
{
    [GeneratedRegex(@"\b(?:ImGui\.(?:TextColored|TextDisabled|TextWrapped|LabelText|BulletText|SetTooltip)|ConsoleWidgets\.(?:Readout|ReadoutPending|RegionHeader|Tooltip|ItemTooltip|WrappedTooltip|BeginTooltipCard))\(")]
    private static partial Regex FormattedCall();

    [GeneratedRegex(@"^\$?""[^""%]*""(?:u8)?(?:\.AsSpan\(\))?$")]
    private static partial Regex PlainLiteral();

    public static readonly string[] Folders = ["Core", "Features/Ui/Console", "Features/Constellations"];

    public static string SourceRoot([CallerFilePath] string here = "") =>
        Path.GetFullPath(Path.Combine(Path.GetDirectoryName(here)!, "..", "..", "AdvancedFlightComputer"));

    public static List<string> Offenders()
    {
        var found = new List<string>();
        string root = SourceRoot();
        foreach (string folder in Folders.Where(folder => Directory.Exists(Path.Combine(root, folder))))
        {
            foreach (string file in Directory.EnumerateFiles(Path.Combine(root, folder), "*.cs", SearchOption.AllDirectories))
            {
                string source = File.ReadAllText(file);
                foreach (string text in UnsafeTexts(source))
                    found.Add(Path.GetRelativePath(root, file).Replace('\\', '/') + ": " + text);
            }
        }
        return found;
    }

    /// <summary>Each text argument of a formatted call in <paramref name="source"/> that may hold a '%'.</summary>
    public static IEnumerable<string> UnsafeTexts(string source)
    {
        foreach (Match m in FormattedCall().Matches(source))
        {
            List<string> args = Arguments(source, m.Index + m.Length);
            foreach (string text in TextArguments(m.Value, args))
            {
                if (!IsSafe(text))
                    yield return m.Value + text + ")";
            }
        }
    }

    // BeginTooltipCard takes its title first, a readout and LabelText format a key and a value, and the other calls take the text last.
    private static IEnumerable<string> TextArguments(string call, List<string> args)
    {
        if (args.Count == 0)
            return args;
        if (call.Contains("BeginTooltipCard", StringComparison.Ordinal))
            return [args[0]];
        if (call.Contains("Readout", StringComparison.Ordinal) || call.Contains("LabelText", StringComparison.Ordinal))
            return args;
        return [args[^1]];
    }

    private static bool IsSafe(string text) =>
        PlainLiteral().IsMatch(text) || text.StartsWith("ConsoleUi.EscapeFormat(", StringComparison.Ordinal)
        || text.StartsWith("EscapeFormat(", StringComparison.Ordinal) || text.StartsWith("escaped", StringComparison.Ordinal);

    private static List<string> Arguments(string source, int start)
    {
        var args = new List<string>();
        int depth = 1, argStart = start;
        bool quoted = false;
        for (int i = start; i < source.Length; i++)
        {
            char c = source[i];
            if (quoted)
            {
                if (c == '\\')
                    i++;
                else if (c == '"')
                    quoted = false;
                continue;
            }
            if (c == '"')
                quoted = true;
            else if (c == '(' || c == '[' || c == '{')
                depth++;
            else if (c == ')' || c == ']' || c == '}')
            {
                depth--;
                if (depth == 0)
                {
                    args.Add(source[argStart..i].Trim());
                    return args;
                }
            }
            else if (c == ',' && depth == 1)
            {
                args.Add(source[argStart..i].Trim());
                argStart = i + 1;
            }
        }
        return args;
    }
}
