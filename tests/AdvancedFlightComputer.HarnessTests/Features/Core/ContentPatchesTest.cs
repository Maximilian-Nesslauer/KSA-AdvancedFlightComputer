using System.Xml.Linq;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The asset files of the deployed mod merge into the stock content with zero content errors, and the
// merged rows hold what the mod depends on: a Mass and an SOI on the interstellar comets, and the
// AUTOSTAGE button on the EngineControl gauge panel.
//
// The harness manifest enables only Core and HeadlessHarness, so the session itself never reads the
// asset files of this mod. The test therefore reads Core and the deployed mod into a separate content
// source through the game's own reader, merges it with the game's own pre-pass, and deserializes the
// merged rows with the game's own deserializer, without registering anything. Core alone goes through
// the same steps, so that a stock content error does not count against the mod.
public sealed class ContentPatchesTest : AfcTest
{
    private const string ModId = "AdvancedFlightComputer";
    private const string ContentRoot = "Content";

    public override string Name => "afc-content-patches";

    protected override void Execute(TestContext t)
    {
        ContentErrors coreErrors = Merge(withMod: false, out _);
        ContentErrors errors = Merge(withMod: true, out List<FlatRow> rows);

        int modErrors = 0;
        for (int i = 0; i < errors.Count; i++)
        {
            ContentError error = errors[i];
            if (!string.Equals(error.Mod, ModId, StringComparison.OrdinalIgnoreCase))
                continue;
            modErrors++;
            t.Info($"content error: {error.Kind} {error.File} <{error.Tag} Id=\"{error.Id}\"> {error.Detail}");
        }
        t.Check("the asset files of the mod read and merge without a content error", modErrors == 0, $"{modErrors} errors");
        t.Check("the mod adds no content error to Core", errors.Count == coreErrors.Count,
            $"Core alone {coreErrors.Count}, with the mod {errors.Count}");

        CheckComet(t, rows, "Oumuamua", gg: 4.19, tg: double.NaN);
        CheckComet(t, rows, "Borisov", gg: 524.0, tg: double.NaN);
        // Stock gives 3I_ATLAS its Mass, and the patch leaves it as it is.
        CheckComet(t, rows, "3I_ATLAS", gg: double.NaN, tg: 44.0);
        CheckEngineControl(t, rows);
    }

    private static ContentErrors Merge(bool withMod, out List<FlatRow> rows)
    {
        ModManifest manifest = new ModManifest();
        manifest.Mods.Add(Enabled("Core", 0));
        if (withMod)
            manifest.Mods.Add(Enabled(ModId, 1));
        ContentErrors errors = new ContentErrors();
        ContentSource source = ContentLoader.ReadSource(ContentRoot, ModLibrary.LocalModsFolderPath, manifest, errors);
        rows = ContentPrePass.Resolve(source.Rows, errors);
        return errors;
    }

    private static ModEntry Enabled(string id, int index) => new ModEntry(id, index) { Enabled = true };

    private static T? Deserialize<T>(TestContext t, List<FlatRow> rows, string tag, string id) where T : SerializedId
    {
        FlatRow? row = rows.Find(r => r.Tag == tag && string.Equals(r.Id, id, StringComparison.OrdinalIgnoreCase));
        if (row == null)
        {
            t.Fail($"{tag} {id} is in the merged content", "no row");
            return null;
        }
        if (!ModLibrary.TagTable.TryGet(tag, out ContentTagBinding binding))
        {
            t.Fail($"{tag} is a bound content tag", "no binding");
            return null;
        }
        ContentErrors errors = new ContentErrors();
        SerializedId? asset = ContentDeserializer.Deserialize(row, binding, ModLibrary.TagTable, null, errors);
        string detail = errors.Count > 0 ? errors[0].Detail : (asset == null ? "null" : asset.GetType().Name);
        t.Check($"{tag} {id} deserializes after the merge", asset is T && errors.Count == 0, detail);
        return asset as T;
    }

    private static void CheckComet(TestContext t, List<FlatRow> rows, string id, double gg, double tg)
    {
        InterstellarCometTemplate? comet = Deserialize<InterstellarCometTemplate>(t, rows, "InterstellarComet", id);
        if (comet == null)
            return;
        t.Check($"{id} has an SOI of 50000 km", comet.SphereOfInfluence.Km == 50000.0, $"Km={comet.SphereOfInfluence.Km}");
        t.Check($"{id} has its Mass", Same(comet.Mass.Gg, gg) && Same(comet.Mass.Tg, tg),
            $"Gg={comet.Mass.Gg} Tg={comet.Mass.Tg}");
        t.Check($"{id} keeps its stock orbit and radius", !double.IsNaN(comet.MeanRadius.Km) && comet.Orbit != null);
    }

    private static bool Same(double actual, double expected) =>
        double.IsNaN(expected) ? double.IsNaN(actual) : actual == expected;

    private static void CheckEngineControl(TestContext t, List<FlatRow> rows)
    {
        GaugeCanvas? canvas = Deserialize<GaugeCanvas>(t, rows, "GaugeCanvas", "EngineControl");
        if (canvas == null)
            return;

        GaugeComponent? stock = canvas.Components.Find(c => c.Name == "Buttons");
        GaugeComponent? afc = canvas.Components.Find(c => c.Name == "AfcAutoStage");
        t.Check("the stock Buttons component keeps its three buttons",
            stock != null && stock.Rects.Count(r => r is GaugeButtonReference) == 3,
            stock == null ? "no Buttons component" : $"{stock.Rects.Count} rects");
        if (!t.Check("the AfcAutoStage component is the last component", afc != null && ReferenceEquals(canvas.Components[^1], afc),
                $"{canvas.Components.Count} components"))
            return;
        // The same shaders put the button into the draw batch of the stock buttons.
        t.Check("the AfcAutoStage component uses the shaders of the stock buttons",
            stock != null && afc!.FragmentShader?.Ref == stock.FragmentShader?.Ref && afc.VertexShader?.Ref == stock.VertexShader?.Ref,
            $"fragment {afc!.FragmentShader?.Ref}, vertex {afc.VertexShader?.Ref}");

        GaugeButtonReference? button = afc.Rects.OfType<GaugeButtonReference>().FirstOrDefault();
        GaugeButtonFlightComputer? binding = button?.Binding as GaugeButtonFlightComputer;
        t.Check("the AUTOSTAGE button toggles AfcAutoStageToggle",
            button != null && button.Label == "AUTOSTAGE" && binding != null
            && binding.EnumType == "AfcAutoStageToggle" && binding.Value == "Enabled" && binding.Toggle,
            binding == null ? "no FlightComputer binding" : $"{binding.EnumType}.{binding.Value} toggle={binding.Toggle}");
        t.Check("the AUTOSTAGE button sits in the lower-left slot", button != null && button.X == 0.006 && button.Y == 0.5,
            button == null ? "no button" : $"X={button.X} Y={button.Y}");
    }
}
