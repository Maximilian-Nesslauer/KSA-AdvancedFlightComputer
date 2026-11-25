# Landing sites for mod authors

A mod can add named landing sites, such as landing pads or a moored landing barge, by placing an `AfcLandingSites.toml` file in its mod folder. AFC reads this file from every enabled mod at load, and neither mod needs a dependency on the other.

- **RETURN TO** in each booster row of **LAUNCH > BOOSTER RETURN** lists the sites on the launch body, grouped and sorted nearest to the launch site first, and assigns each site to at most one booster.
- **LAND AT** under the site row of **LAND** lists sites sorted nearest to the craft first, and marks reachable sites while the craft is falling. Selecting a site sets the same target as **PICK SITE**.

```toml
[[site]]
id = "ExampleMod.LZ.A"             # unique id, required
name = "LZ A"                      # name in the lists, required
group = "Example launch complex"   # optional heading in the lists
system = "SolLite"                 # optional, the site shows only in this system
body = "Earth"                     # required
latitude = 28.6072                 # degrees, body-fixed, required
longitude = -80.6078               # degrees, body-fixed, required
height = "terrain"                 # "terrain" (default) or "fixed"
# heightAboveMeanRadius = 3.35     # metres, required with height = "fixed"
usableRadius = 30.0                # metres, default 20 (> 0 and <= 500)
```

## Field rules

- A `terrain` site is a pad on the ground, and AFC queries its height from the terrain.
- A `fixed` site is a deck at `heightAboveMeanRadius` above the body's mean radius, for a barge or offshore platform where the terrain query returns the sea.
- A target within `usableRadius` (`> 0` and `<= 500` m, default `20` m) of a site counts as on that site.
- Duplicate `id` values are skipped (the first loaded entry wins).
- Entries with missing or invalid values, or whose `system` or `body` is not loaded, are skipped with one line each in the game log.
