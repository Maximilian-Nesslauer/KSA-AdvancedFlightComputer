using System;
using System.Text;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class PartNames
{
    private const string PrefabMark = "_Prefab_";

    public static string Display(string name)
    {
        int mark = name.IndexOf(PrefabMark, StringComparison.Ordinal);
        if (mark < 0)
            return name;
        string tail = name.Substring(mark + PrefabMark.Length);
        var text = new StringBuilder(tail.Length + 4);
        for (int i = 0; i < tail.Length; i++)
        {
            char c = tail[i];
            if (c == '_')
            {
                text.Append(' ');
                continue;
            }
            bool afterLower = i > 0 && char.IsLower(tail[i - 1]) && (char.IsUpper(c) || char.IsDigit(c));
            bool endsCapitals = i > 0 && i + 1 < tail.Length && char.IsUpper(tail[i - 1]) && char.IsUpper(c) && char.IsLower(tail[i + 1]);
            if (afterLower || endsCapitals)
                text.Append(' ');
            text.Append(c);
        }
        return text.ToString();
    }
}
