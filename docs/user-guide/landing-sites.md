# Landing sites for mod authors

A mod can add named landing sites, such as landing pads or a moored landing barge, in a file `AfcLandingSites.toml` in its own folder. AFC reads this file from every enabled mod at load, and neither mod needs the other. **RETURN TO** in each booster row of **LAUNCH > BOOSTER RETURN** lists the sites of the launch body, grouped and nearest to the launch site first, and gives one site to one booster only. **LAND AT** under the site row of **LAND** lists them nearest to the craft first, and while the craft falls it marks the sites in reach. A pick sets the same target as **PICK SITE**.

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
usableRadius = 30.0                # metres, default 20
```

A `terrain` site is a pad on the ground, and AFC reads its height from the terrain. A `fixed` site is a deck at `heightAboveMeanRadius` above the mean radius of the body, for a barge or a platform in the water, where the terrain query gives the sea. A target within `usableRadius` of a site is on that site. AFC leaves out an entry with a missing or bad value, and a site whose system or body is not loaded, with one line in the game log each.
