namespace AdvancedFlightComputer.Features.MultiPass;

internal static class TomlIo
{
    public static string Escape(string s) =>
        s.Replace("\\", "\\\\").Replace("\"", "\\\"");

    public static string Unescape(string s) =>
        s.Replace("\\\"", "\"").Replace("\\\\", "\\");
}
