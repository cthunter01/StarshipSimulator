# StarshipSimulator

A walk-around simulator for space habitats (O'Neill cylinders, Bishop rings, starships) under an accurate sky.
C++23, CMake presets + Ninja, GoogleTest, SDL3 + SDL_GPU (Vulkan, Metal), Dear ImGui, Jolt Physics.
Cross-platform: Linux (GCC, Clang), macOS (Apple Clang) and Windows (MSVC). The app runs on Vulkan (Linux,
Windows) and Metal (macOS), where each SPIR-V shader is translated to MSL as it loads.
Roadmap and design decisions: `~/.claude/plans/i-want-to-create-compressed-wreath.md` (M0 foundation → M10).

## Commands
- Build and test (Clang Debug): `cmake --workflow --preset dev`
- Rebuild only: `cmake --build --preset clang-debug`
- Test only: `ctest --preset clang-debug`
- One test: `ctest --preset clang-debug -R 'LookRig\.'` or `build/clang-debug/bin/StarshipSimulator_tests --gtest_filter='LookRig.*'`
- Run: `build/clang-debug/bin/StarshipSimulator` (`--help` for options)
- Before finishing a change, also run: `cmake --workflow --preset tidy` (clang-tidy, warnings are errors),
  `cmake --workflow --preset asan` (AddressSanitizer + UBSan) and `cmake --workflow --preset ci-gcc`
  (GCC-only warnings such as `-Wuseless-cast`)
- On Windows the presets are `msvc-debug` (workflow `dev-msvc`), `msvc-release` and `ci-msvc`, and cmake must run
  in a Developer PowerShell for VS (glslc comes from the Vulkan SDK). `tidy`, `asan`, `tsan` and `coverage` exist
  on Linux and macOS only
- Formatting is automatic: a Claude Code hook (`.claude/hooks/format-cpp.sh`) runs clang-format on every C/C++
  file right after you edit it. The pre-commit hook and CI also reject unformatted files

Other presets: `clang-release`, `gcc-debug`, `gcc-release`, `tsan`, `coverage`, `ci-gcc`, `ci-clang`.
Each builds into `build/<preset>/`; never edit anything under `build/`. A preset is only available on the
platforms it supports (`gcc-*`: Linux; `clang-*`: Linux and macOS; `msvc-*`: Windows); `cmake --list-presets`
shows this machine's. CI (`.github/workflows/ci.yml`) runs `ci-gcc`, `ci-clang`, `asan` and `tidy` on Linux,
`ci-clang` on macOS and `ci-msvc` on Windows, and checks formatting. The macOS runners have a Metal GPU, so that
job also runs the app: it renders the town and valley views with Metal's API validation on (any misuse fails the
job) and keeps the pictures with the run.

## Checking visuals yourself
The app can render and save a screenshot without interaction, then exit:
`build/clang-debug/bin/StarshipSimulator --size 1280x720 --view lookup [--mirror 30] --capture out.png [--capture-ui]`
Views: valley, river, lake, town, street, rooftops, tram, lift, hub, lookup, window, endcap, ramp, sunward,
axis, overview (or `--camera x,y,z,yaw,pitch` in the
habitat frame; `--scenario data/presets/coriolis_playground.toml` for the small habitat). Write captures to the
scratchpad and inspect them with the Read tool before reporting visual work as done. The window opens briefly
on the user's desktop (Wayland). Add `--no-gpu-debug` for quicker runs. There is no Mac here: to see what Metal
draws, push and fetch the pictures of the macOS CI job, `gh run download <run id> -n metal-pictures`
(`gh run list` gives the id)
- The panels: `--panel editor|gallery|almanac` opens one at startup (repeatable), which is the only way
  to get them into a capture; `--capture-ui` then includes them. `--tour 1|2|3` (or part of a tour's
  name) sets off on a guided tour, and captures step 1/60 s per frame, so `--capture-frames 1800`
  lands 30 s into it
- The weather: `--weather clear|fair|cloudy|overcast|mist|rain|storm` holds it still (otherwise it runs itself
  from the clock); the season follows the date, so `--time 2045-07-21T10:00` is autumn in the Island Three
  preset. Captures and `--benchmark` are always silent; `--mute` silences an ordinary run
- The sky: `--time 2045-06-15T01:00` (UTC; the day schedule makes 20:00-06:00 night), `--look-at earth|moon|
  jupiter|Vega|partner` (spins the habitat so it shows through window 0 and floats you off the axis facing it),
  `--fov 2.5` to zoom in. Captures step a fixed 1/60 s per frame and load the sky data synchronously, so they
  are repeatable; keep the window size small (e.g. 960x540), the compositor may resize large windows
- Photo mode (F2) hides the HUD, frees the camera and offers a long exposure and an enlarged
  picture. The exposure keeps the brightest each pixel has been (`BlendMode::LIGHTEN` into a
  separate target), so the stars draw arcs as the habitat turns while everything still stays sharp;
  an enlarged picture renders the scene at 2-4x and scales it down. The two do not combine: an
  exposure is held at one size, so while one is running the picture comes out at the window's size
- Performance: `build/clang-release/bin/StarshipSimulator --size 1920x1080 --no-vsync --no-gpu-debug --benchmark`
  prints average/p99 frame times over a fixed tour and per view (M3 on the RTX A1000: ~6 ms average, ~13 ms
  p99; towns and physics in M4 added about 1 ms; M6, with the clouds, the crowds and the tramway, measures
  7.3 ms average and p99 15 ms at 1080p, or 7.1 / 14.9 under thick cloud; the goal is p99 < 20 ms). Another
  app instance running (uncapped, in mailbox mode) halves the GPU and ruins the numbers: check `nvidia-smi`
  first. Frame times sometimes come out quantized to whole refresh intervals (6.06 ms here) because the
  compositor is pacing the presents, which inflates everything and hides small differences: if every view's
  average is a multiple of the refresh interval, the numbers are not measuring the renderer. Compare two
  builds under the same conditions (a `git worktree` of the previous commit) rather than against older
  numbers, and check `nvidia-smi dmon` to see whether the GPU is the limit at all. The log
  reports the terrain, tree and town GPU memory at startup
- ASan build of the app:
  `LSAN_OPTIONS=suppressions=tools/lsan.supp:fast_unwind_on_malloc=0 build/asan/bin/StarshipSimulator --no-gpu-debug ...`
  (system libraries such as libdbus leak on purpose; the Vulkan validation layer leaks a few hundred bytes of its
  own bookkeeping, so leave GPU debug off for leak checks; the NVIDIA driver keeps a few bytes per pipeline, which
  only the slow unwinder can attribute)

## Layout
- `include/StarshipSimulator/<module>/`: public headers; `src/<module>/`: sources
- `core` (`StarshipSimulator_core`): everything that can be unit tested without a GPU. **No SDL, ImGui, Jolt,
  physics or render includes**: the `layering` test (`cmake/CheckLayering.cmake`) fails otherwise
  - `habitat/`: `OneillCylinderSpec` (the shareable description), `metrics` (spin, gravity, air, hull strength),
    `MeridianProfile` (the revolved cross-section), `HabitatGeometry` (regions, terrain, ground queries, water,
    forest density), `Landscape` (rivers, lakes, shore shaping, woodland), `mirror_optics` (where the sun
    appears, day/night, which beam lights a point), `day_schedule` (mirror angle by local time),
    `weather` (`ClimateSpec`, `weatherAt`: cloud, rain, wetness, mist, wind and the season as a smooth function
    of time, never simulated or remembered, so two people at the same moment see the same sky; `seasonalDay`
    stretches the day schedule over the year)
  - `tour.h`: guided tours (`habitatTours(geometry, name)` builds them for the habitat you are in,
    `tourAt` gives the eye, the angles and the caption at a moment). Stops hold a viewpoint, a
    caption, how long the move to it takes and how long it is held, and may set the mirror angle,
    the weather or the time scale. Every caption and every distance is worked out from the habitat
    (the tours must also fit a 250 m cylinder, which `tour_tests` checks), and
    `place(geometry, z, theta, height)` puts a stop that many metres above the floor -- heights are
    positive, the radius shrinks -- nudging it to the nearest open ground so no trunk stands in
    the frame
  - `almanac.h`: the pages the almanac shows (K in the app) -- what the habitat is, in numbers and
    plain words, worked out from the habitat you are standing in and the moment you are in it.
    `dropDeflection` and `jumpDeflection` fly the simulation's own free flight rather than quoting
    a formula, so the book and the world cannot disagree
  - `astro/`: `SimTime` (int64 microseconds since J2000, UT), `ephemeris` (Astronomy Engine: bodies, Lagrange
    point locations, `computeSky`, `habitatFromEqj`: the spin axis points at the Sun), `star_catalog` (HYG),
    `sky_objects` (phases, naming what the crosshair points at)
  - `assets/`: file reading, gzip/zlib, a minimal OpenEXR reader (NASA's Milky Way map), JPEG/PNG via stb
  - `physics/`: `RotatingFrame` (centrifugal + Coriolis, exact free flight, `stepFreeBody`), `PlayerController`
    (walk/fly/wings logic; collisions through a `CharacterMover`, or the bare analytic terrain without one),
    `colliders.h` (static boxes and hulls as plain data, `floorOrientation`). `Locomotion::WINGS` is a real
    aerofoil in the habitat's air (`airDensityAt` in `habitat/metrics`): it glides at one gravity but only
    climbs on muscle power up near the axis, which is the whole point of the place
  - `procgen/`: deterministic noise, `terrain_grid` (the valley floor and endcaps sampled into a height field
    and land-cover map, the GPU's source), `TerrainLod` (CDLOD quadtree: patches and morph ranges),
    `trees` (procedural species meshes, planting in tiles), `habitat_mesher` (chunked meshes; the app
    meshes only the glass and end walls, the landscape pass draws the land), `hull_mesh` (the outside, for the
    partner cylinder, and `partnerTransform`), placeholder star field, mesh primitives,
    `settlements` (towns and farms planned on the unrolled floor: streets that follow the river, lots,
    houses, square, bridge, river front, props, town trees, 1 m ground maps; `stampSettlements` marks them
    in the cover map), `buildings` (their meshes and colliders; facades are packed into
    `Vertex::material` and drawn by the shader), `props` (balls, crates, barrels, bales, cafe furniture:
    meshes and collision parts), `clouds` (the cloud map: cover and detail wrapped once round the habitat and
    once along it), `birds` (flocks wheeling over fixed places on the floor, worked out fresh each frame),
    `people` (who is walking which street, sitting on which bench, and the one body mesh they all share),
    `transit` (a tramway down each valley calling at its towns, a funicular up each endcap's ramp to the
    hub, the track's mesh in chunks, and where every car is at a moment). A line's alignment is
    smoothed into something buildable -- as straight as it can be inside a band around the ground,
    then rounded into vertical curves -- and `gradeForTrack` cuts and fills the terrain grid to it,
    with side slopes at a constant angle and the woods cleared. Where the ground falls more than a
    few metres below the alignment it is left alone and the track goes on a trestle instead. The
    grading runs before the towns, the woods and the level-of-detail tree are built, so everything
    else stands on the ground the railway left
  - `audio/`: `Soundscape`, the synthesised ambience (wind, leaves, water, rain, birds or crickets, a town's
    murmur, footsteps). It fills a buffer of frames and knows nothing about devices, so it is unit tested
  - The terrain and settlement sources are compiled with `-O2` even in Debug (`src/core/CMakeLists.txt`),
    or world generation takes several seconds
  - `scenario/`: TOML habitat files (toml++, used only in `scenario.cpp`; bump `kGeneratorVersion` when
    generation changes; `[climate]` holds the cloud deck, how often it rains and the length of the year,
    `season_at_epoch` deciding where in that year J2000 falls, and so which season a preset starts in).
    `validateScenario` returns every problem with a whole scenario as a sentence (the editor lists them
    all while you drag sliders; `parseScenario` reports the first); `describeHabitat` sums one up in a
    line for the gallery;
    `gpu_abi/`: uniform structs, SPIR-V reflection, `metal_shader` (SPIR-V to MSL for SDL_GPU on Metal, by
    SPIRV-Cross, with the resources renumbered to SDL's Metal layout), `color_grade` (the grading LUT),
    `ground_atlas` (the towns' ground maps packed for the landscape shader); plus camera, frustum, sim
    clock, app options, `parse_number` (`parseInt`/`parseDouble`: whole-text, locale-free; Apple's libc++ has
    no floating-point `std::from_chars`, so it falls back to `strtod` there) and `utf8_path` (see Conventions)
- `physics` (`StarshipSimulator_physics`): `PhysicsWorld`, Jolt Physics v5.6.0 in double precision behind a
  pimpl (Jolt is linked PRIVATE: its target exports `-mavx2` and its config defines). Jolt's gravity is
  zero: the `SpinFrame` step listener gives moving bodies the centrifugal kick and turns their velocity for
  Coriolis (leapfrog: launched bodies get a half kick), plus buoyancy in water. Terrain collision is
  built from the terrain grid in tiles (the drawn triangles) around the player and moving props, tree
  trunks per tree tile near the player; the player is a `CharacterVirtual` stood along the local up
  every step, and inherits the velocity of whatever it is standing on, so a tram carries you. `setPeople`
  and `setTrams` keep pools of kinematic bodies where the nearest people and cars are, so you bump into
  them and can ride on them. No SDL or render includes (layering test). Include Jolt only through
  `src/physics/jolt.h`
  (Jolt.h must come first). Jolt is built with `-O2` even in Debug. Unit tests link core and physics
- `audio` (`StarshipSimulator_audio`): `AudioDevice`, SDL's audio device pulling frames from `core/audio`'s
  `Soundscape` on its own thread (a mutex guards it). No renderer, ImGui or Jolt includes (layering test); a
  machine without sound just stays quiet
- `render` (`StarshipSimulator_render`): SDL_GPU device, shader library, `GpuWorld` (all habitat chunks in one
  vertex/index buffer), `GpuLandscape` (height, cover and profile textures, the instanced CDLOD patch mesh),
  `GpuTrees` (instances by tile, two detail levels), `GpuPeople` (one body mesh, bent into a stride by
  `person.vert`), `GpuTransit` (the track in chunks and the tram car, instanced),
  `ShadowMap` (trees, along the dominant mirror beam),
  `DynamicBuffer` (per-frame storage uploads), `GpuSettlements` (town meshes, ground-map atlas), `GpuProps`
  (prop meshes, per-frame instances), `GpuBirds` (this frame's birds, no mesh: six vertices each in the
  shader), passes (Milky Way, stars, planets, Earth/Moon, partner hull, terrain disks, landscape, trees,
  buildings, props, people, transit, mirrors, birds, markers, water, glass, clouds, rain, tonemap with the
  grade LUT; drawn in that order after the shadow pass, which holds trees, buildings, props, people and the
  tramway), `texture.h` (2D uploads with
  GPU mipmaps), `pipeline.h` helper, ImGui layer
  - The cloud deck is ray-marched, so it is drawn on a stand-in cylinder just outside the deck's base
    (`cloud_shell.vert`) rather than on a full-screen triangle, and writes no depth: the depth test then
    throws away every pixel with land in front before the march runs, and `layout(early_fragment_tests)`
    makes sure it happens early. From below the deck the pass draws the faces where rays enter that cylinder,
    from inside it the faces where they leave, so each pixel is shaded exactly once
- `src/app/`: the `StarshipSimulator` executable (main loop, input, HUD, editor, gallery, sky data loading
  on a worker thread); its headers are private. The editor holds a whole `Scenario` as its draft
  (shape, day, land, air, place and time), not just the habitat's shape: nothing reaches the world
  until "Build it", which is `buildDraft`. The gallery reads every preset and saved file at startup
  (and on "Look again"), so a broken file shows its error rather than going missing. `walkTo` stands the player on the terrain grid rather than the analytic
  terrain: the two differ wherever the tramway has graded the land. The people, the birds and the
  trams are worked out fresh every frame from
  `animationSeconds` and handed to both the renderer and the physics. The weather and the season come from
  `weatherAt` each frame; the HUD's sliders
  follow them until "Hold the weather" is ticked, so holding starts from what is outside the window. The
  clouds drift, the birds fly and the rain falls in real time (like the spin), from `animationSeconds`
- `third_party/`: vendored C code (Astronomy Engine, stb) in their own targets, never reformatted
- `cmake/SkyData.cmake`: downloads the star catalog and sky maps (~52 MB, SHA-256 checked) once into
  `build/_downloads/sky`; off when `CI` is set. Tests using them skip when missing. Credits: `data/CREDITS.md`
- `shaders/`: GLSL, compiled by glslc to `build/<preset>/bin/shaders/*.spv` (`cmake/Shaders.cmake`);
  `include/habitat.glsl` holds the shared air (aerial perspective) and sunlight (mirror beams) models;
  `include/clouds.glsl` the cloud deck (its map, cover, density and the shade it casts) and
  `include/clouds_shell.glsl` the stand-in cylinder's radius (keep `SHELL_SEGMENTS` in step with `CloudPass`);
  `include/landscape.glsl` the height field and terrain shadow march, `include/shadow.glsl` the tree shadow
  lookup, `include/people.glsl` which part of a body a vertex belongs to (must match `person_part`),
  `include/terrain_colors.glsl` the fields, meadows, shore and town colours, `include/lit.glsl` the
  light on buildings and props (hill and shadow-map shadows, lights at night), `include/town_ground.glsl`
  the ground-map lookup (take screen derivatives before calling: it samples in divergent control flow)
- `data/presets/*.toml`: scenario presets, copied to `build/<preset>/bin/data` at build time
- `tests/`: GoogleTest files, all in `StarshipSimulator_tests` (a class's tests: `MyClassTests.cpp`, e.g.
  `SimClockTests.cpp`; other tests: `*_tests.cpp`)
- `cmake/ProjectOptions.cmake`: `StarshipSimulator_configure_target()` (warnings, sanitizers, coverage, tidy)
- `cmake/Dependencies.cmake`: third-party libraries via FetchContent (installed packages win, except toml++: always
  header-only from source, as a packaged shared library's exceptions are not caught across it on macOS; and
  SPIRV-Cross, pinned so every platform translates shaders the same way)

## Conventions
- Headers are `.h` (never `.hpp`) and use `#pragma once`
- A header devoted to one class is named exactly after the class, including capitalization, and so is its source:
  `class HabitatGeometry` lives in `include/StarshipSimulator/core/habitat/HabitatGeometry.h` and
  `src/core/habitat/HabitatGeometry.cpp`, and tests of it alone in `tests/HabitatGeometryTests.cpp`. Headers that
  gather several types or free functions keep a snake_case topic name (`camera.h`, `settlements.h`, the
  `passes/*_passes.h` groups of small pass classes)
- Names (clang-tidy's `readability-identifier-naming` enforces them): types and template parameters `CamelCase`;
  enum constants `UPPER_CASE` (`Locomotion::WINGS`); functions, variables and members `camelBack`, private and
  protected members with a trailing `_`; constants (`constexpr`, and `static const` including function-local
  caches) `kCamelCase`; mutable statics `s_name`, mutable globals `g_name`; namespaces `lower_case`, except
  `StarshipSimulator` itself
- Code lives in `namespace StarshipSimulator`; project includes use quotes: `#include "StarshipSimulator/core/camera.h"`
- Every new target of ours must call `StarshipSimulator_configure_target(<target>)`; third-party targets must not
- New source files go into the relevant `CMakeLists.txt`; new tests go into `tests/CMakeLists.txt`; new shaders
  into `shaders/CMakeLists.txt`
- Warnings are part of the build: code must compile cleanly with `-Werror` under GCC and Clang and with `/WX`
  under MSVC
- Code must build and pass its tests on Linux, macOS and Windows (CI runs all three). Use the standard library
  (`<filesystem>`, `<thread>`, `<chrono>`) over POSIX or Win32 APIs; when an OS API is unavoidable, keep it in one
  source file behind an `#ifdef _WIN32` / `__APPLE__` / `__linux__` split, with a branch for each platform. A
  runtime check through SDL (`SDL_GetPlatform()`) is fine in code that already uses SDL
- Text is UTF-8 everywhere: SDL, ImGui, command-line arguments (SDL's main converts them on Windows) and our
  messages. Convert paths with `pathFromUtf8` / `utf8String` (`core/utf8_path.h`), never
  `std::filesystem::path(const char*)` or `path::string()`, which use the ANSI code page on Windows. Numbers
  read from text go through `parseInt` / `parseDouble` (`core/parse_number.h`)
- Math: use the aliases in `core/math.h` (`Vec3d`, `Mat4f`, ...), never raw `glm::` types. GLM is configured
  with `GLM_FORCE_EXPLICIT_CTOR`, so double → float conversions must be explicit
- Habitat frame: spin axis +Z, the Sun toward +Z; window i is centred on angle i * 2pi/strips, land strip i halfway
  to the next window; "up" is toward the axis. Everything is simulated in this rotating frame
- Sky frame: J2000 equatorial (EQJ). `astro::habitatFromEqj(sunDirection, spinPhase)` turns it into the habitat
  frame; shaders get only that matrix (never times). Simulated time (`SimTime`) is separate from the spin, which
  always runs in real time
- Determinism: procedural generation uses our own `SplitMix64`/`SimplexNoise` (never `std::` distributions) and
  core builds with `-ffp-contract=off` (MSVC does not contract under its default `/fp:precise`);
  `buildHabitatMeshes` output must not depend on the thread count. Nothing generated may depend on what a
  standard library leaves to the implementation: the order `std::sort` leaves equal elements in (break ties),
  `unordered_*` iteration order, `std::hash`, `std::shuffle`.
  **One random draw to a statement**: C++ does not say which argument of a call (or which operand of
  `+`) is worked out first, so `f(rng.uniform(), rng.uniform())` hands the two values over in
  whichever order the compiler chose, and the same habitat file then grows a different town under a
  different compiler. Put each draw in its own named variable first. `determinism_tests` hashes a
  whole generated world against one value for every compiler and standard library CI runs (GCC and
  Clang with libstdc++, Apple Clang with libc++, MSVC); when generation changes on purpose, bump
  `Scenario::kGeneratorVersion` and record the new hash
- Precision: world positions are `double`. The GPU only sees camera-relative `float` data (compute
  `position - camera` in double, then convert). Depth is reverse-Z with an infinite far plane: clear to 0,
  compare `GREATER`
- Logging: `core/log.h` (`log::info/warn/error`, std::format). No printf-style varargs anywhere: for ImGui text
  use `ui::text/field/textWrapped(std::format(...))` from `render/ImGuiLayer.h`, never `ImGui::Text("%...")`
  (the one exception is Jolt's `Trace` callback, which only logs its format string)
- `SDL_Event` is a union: read it only inside `SdlInput::handleEvent` (`src/app/SdlInput.cpp`, NOLINT region)
- SDL_GPU create-info structs use designated initializers naming only the fields that matter (the render target
  disables the missing-field warning for this)

## SDL_GPU shader rules (Vulkan / SPIR-V, and Metal)
Shaders are written and compiled once, as GLSL for Vulkan. On Metal (macOS) `ShaderLibrary` translates each
SPIR-V shader to MSL as it loads (`gpu::translateToMetal`): SPIRV-Cross writes MSL 2.1 whose resources are
renumbered the way SDL_GPU binds them on Metal ([[texture]] sampled then storage textures, [[sampler]] one per
sampled texture, [[buffer]] uniform then storage buffers; vertex buffers are [[stage_in]], from [[buffer(14)]]),
and the entry point becomes `main0`. `metal_shader_tests` translates every built shader. SDL compiles MSL with
Metal's default options, which include fast math. There are no compute shaders yet; translating them is refused.
Resource bindings must use SDL_GPU's descriptor sets; within a set, bindings count up from 0 in this order:

| Stage    | Textures and storage                                                    | Uniform buffers |
|----------|-------------------------------------------------------------------------|-----------------|
| vertex   | set 0: sampled textures, then storage textures, then storage buffers   | set 1           |
| fragment | set 2: same order                                                       | set 3           |
| compute  | set 0: sampled, read-only storage textures, read-only storage buffers; set 1: read-write storage textures, read-write storage buffers | set 2 |

- Only combined `sampler2D` (no separate texture/sampler objects), no push constants, no arrays of resources.
  Storage resources in vertex/fragment shaders must be `readonly`
- A uniform block's binding is its push slot: `SDL_PushGPUFragmentUniformData(cmd, <binding>, ...)`
- Uniform blocks use only `vec4`/`mat4` members (std140); mirror each in `core/gpu_abi/uniforms.h` with
  `alignas(16)` and a `static_assert` on the size
- Resource counts are reflected from the SPIR-V (`ShaderLibrary::load`), never written by hand. The
  `SpirvReflect.AllBuiltShadersFollowTheSdlGpuLayout` test validates every compiled shader
- Shaders are compiled for `--target-env=vulkan1.0 -fpreserve-bindings`: without the latter, optimized (Release)
  builds drop unused uniform blocks and break the contiguous slot numbering. F5 in the app hot-reloads shaders
- Include order for uniforms: `frame.glsl` (slot 0) then `habitat.glsl`, `sky.glsl` or `body.glsl` (slot 1);
  define `UNIFORM_SET` first
- Colour textures from 8-bit images use `*_UNORM_SRGB` formats (decoded by the sampler); HDR maps use RGBA16F

## GPU notes (this machine)
- Laptop with an NVIDIA RTX A1000 (4 GB) and Intel Iris Xe; only the NVIDIA GPU has a Vulkan driver installed.
  The device prefers the discrete GPU; the HUD and the startup log show which GPU is used. If the wrong one is
  picked: `VK_LOADER_DRIVERS_SELECT='nvidia*'`
- Vulkan validation runs in Debug builds (`--gpu-debug` / `--no-gpu-debug`) when `vulkan-validation-layers`
  is installed
