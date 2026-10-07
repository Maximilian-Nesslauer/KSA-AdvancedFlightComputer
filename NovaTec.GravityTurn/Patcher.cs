using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace NovaTec.GravityTurnMod
{
    [HarmonyPatch]
    internal static class Patcher
    {
        private static Harmony? _harmony = new Harmony("NovaTec.GravityTurn");

        public static void Patch()
        {
            Console.WriteLine("Patching GravityTurn...");
            _harmony?.PatchAll(typeof(Patcher).Assembly);
        }

        public static void Unload()
        {
            _harmony?.UnpatchAll(_harmony.Id);
            _harmony = null;
        }

        [HarmonyPatch(typeof(ModLibrary), nameof(ModLibrary.LoadAll))]
        [HarmonyPostfix]
        public static void AfterLoad()
        {
            Console.WriteLine("ModLibrary.LoadAll patched by GravityTurn.");
        }
    }

}
