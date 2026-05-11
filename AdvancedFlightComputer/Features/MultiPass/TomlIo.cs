namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// String escape/unescape for TOML basic strings (the format
/// MultiPassRegistry uses for vehicle ids, intent kinds, etc.).
/// Shared by intent serializers so each one doesn't roll its own.
/// </summary>
internal static class TomlIo
{
    public static string Escape(string s) =>
        s.Replace("\\", "\\\\").Replace("\"", "\\\"");

    public static string Unescape(string s) =>
        s.Replace("\\\"", "\"").Replace("\\\\", "\\");
}
