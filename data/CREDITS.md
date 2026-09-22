# Credits

StarshipSimulator's sky is built from open data and open-source code. Thank you to everyone who made it
available. The same credits are shown in the app (HUD, "Credits").

## Data (downloaded at configure time, not stored in the repository)

| What | Source | License |
| --- | --- | --- |
| Stars (`hyg_v44.csv.gz`) | HYG database v4.4 by David Nash (astronexus), combining Hipparcos, the Yale Bright Star Catalog and the Gliese Catalog of Nearby Stars. https://codeberg.org/astronexus/hyg | CC BY-SA 4.0 |
| Milky Way (`milkyway_2020_4k.exr`) | NASA/Goddard Space Flight Center Scientific Visualization Studio, *Deep Star Maps 2020* (SVS 4851), using Gaia DR2 (ESA/Gaia/DPAC). https://svs.gsfc.nasa.gov/4851 | NASA media guidelines (free to use with credit) |
| Earth by day (`earth_day_2048.jpg`) | NASA Visible Earth, *The Blue Marble: Land Surface, Ocean Color, Sea Ice and Clouds* (2002), Reto Stöckli and Robert Simmon, NASA Goddard Space Flight Center. https://visibleearth.nasa.gov/images/57735 | NASA media guidelines |
| Earth at night (`earth_night_3600.jpg`) | NASA Earth Observatory, *Earth at Night 2012* (Black Marble), Suomi NPP VIIRS. https://visibleearth.nasa.gov/images/79765 | NASA media guidelines |
| Moon (`moon_1k.jpg`) | NASA Scientific Visualization Studio, *CGI Moon Kit* (SVS 4720), from the Lunar Reconnaissance Orbiter Camera (NASA/GSFC/Arizona State University). https://svs.gsfc.nasa.gov/4720 | NASA media guidelines |

The files are checked against SHA-256 hashes after downloading (`cmake/SkyData.cmake`). Without them the app
falls back to placeholder stars and plain-coloured Earth and Moon.

## Code (vendored in `third_party/`)

| What | Source | License |
| --- | --- | --- |
| Astronomy Engine v2.1.19 by Don Cross: positions of the Sun, Moon and planets, Lagrange points, rotation axes, constellations | https://github.com/cosinekitty/astronomy | MIT |
| stb_image and stb_image_write by Sean Barrett: JPEG/PNG decoding, zlib inflate | https://github.com/nothings/stb | MIT or public domain |

Positions are checked against JPL Horizons (DE441) in `tests/astro_tests.cpp`: within one arcminute for the
Moon, Mars, Jupiter and Earth from 1900 to 2100.

The physics (collisions, the walking character, loose props) is Jolt Physics v5.6.0 by Jorrit Rouwe,
https://github.com/jrouwe/JoltPhysics, MIT license, downloaded at configure time.

Other libraries (SDL3, Dear ImGui, GLM, toml++, GoogleTest) are listed in `cmake/Dependencies.cmake`.
