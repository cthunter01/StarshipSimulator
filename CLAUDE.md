# StarshipSimulator

A walk-around simulator for space habitats (O'Neill cylinders, Bishop rings, starships) under an accurate sky.
C++23, CMake presets + Ninja, GoogleTest, SDL3 + SDL_GPU (Vulkan), Dear ImGui, Jolt Physics. Linux, GCC and
Clang.
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
- Formatting is automatic: a Claude Code hook (`.claude/hooks/format-cpp.sh`) runs clang-format on every C/C++
  file right after you edit it. The pre-commit hook and CI also reject unformatted files

Other presets: `clang-release`, `gcc-debug`, `gcc-release`, `tsan`, `coverage`, `ci-gcc`, `ci-clang`.
Each builds into `build/<preset>/`; never edit anything under `build/`.

## Checking visuals yourself
The app can render and save a screenshot without interaction, then exit:
`build/clang-debug/bin/StarshipSimulator --size 1280x720 --view lookup [--mirror 30] --capture out.png [--capture-ui]`
Views: valley, river, lake, town, street, rooftops, lookup, window, endcap, ramp, sunward, axis, overview (or `--camera x,y,z,yaw,pitch` in the
habitat frame; `--scenario data/presets/coriolis_playground.toml` for the small habitat). Write captures to the
scratchpad and inspect them with the Read tool before reporting visual work as done. The window opens briefly
on the user's desktop (Wayland). Add `--no-gpu-debug` for quicker runs.
- The weather: `--weather clear|fair|cloudy|overcast|mist|rain|storm` holds it still (otherwise it runs itself
  from the clock); the season follows the date, so `--time 2045-07-21T10:00` is autumn in the Island Three
  preset. Captures and `--benchmark` are always silent; `--mute` silences an ordinary run
- The sky: `--time 2045-06-15T01:00` (UTC; the day schedule makes 20:00-06:00 night), `--look-at earth|moon|
  jupiter|Vega|partner` (spins the habitat so it shows through window 0 and floats you off the axis facing it),
  `--fov 2.5` to zoom in. Captures step a fixed 1/60 s per frame and load the sky data synchronously, so they
  are repeatable; keep the window size small (e.g. 960x540), the compositor may resize large windows
- Performance: `build/clang-release/bin/StarshipSimulator --size 1920x1080 --no-vsync --no-gpu-debug --benchmark`
  prints average/p99 frame times over a fixed tour and per view (M3 on the RTX A1000: ~6 ms average, ~13 ms
  p99; towns and physics in M4 added about 1 ms; M5's clouds cost a few ms wherever sky fills the view, about
  12 ms average and p99 19 ms at 1080p under thick cloud; the goal is p99 < 20 ms). Another app instance running
  (uncapped, in mailbox mode) halves the GPU and ruins the numbers: check `nvidia-smi` first. Mailbox present
  mode paces frames to whole refresh intervals (6.06 ms here), so small differences hide: compare two builds
  under the same conditions (a `git worktree` of the previous commit) rather than against older numbers, and
  check `nvidia-smi dmon` to see whether the GPU is the limit at all. The log
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
    forest density), `landscape` (rivers, lakes, shore shaping, woodland), `mirror_optics` (where the sun
    appears, day/night, which beam lights a point), `day_schedule` (mirror angle by local time),
    `weather` (`ClimateSpec`, `weatherAt`: cloud, rain, wetness, mist, wind and the season as a smooth function
    of time, never simulated or remembered, so two people at the same moment see the same sky; `seasonalDay`
    stretches the day schedule over the year)
  - `astro/`: `SimTime` (int64 microseconds since J2000, UT), `ephemeris` (Astronomy Engine: bodies, Lagrange
    point locations, `computeSky`, `habitatFromEqj`: the spin axis points at the Sun), `star_catalog` (HYG),
    `sky_objects` (phases, naming what the crosshair points at)
  - `assets/`: file reading, gzip/zlib, a minimal OpenEXR reader (NASA's Milky Way map), JPEG/PNG via stb
  - `physics/`: `RotatingFrame` (centrifugal + Coriolis, exact free flight, `stepFreeBody`), `PlayerController`
    (walk/fly logic; collisions through a `CharacterMover`, or the bare analytic terrain without one),
    `colliders.h` (static boxes and hulls as plain data, `floorOrientation`)
  - `procgen/`: deterministic noise, `terrain_grid` (the valley floor and endcaps sampled into a height field
    and land-cover map, the GPU's source), `terrain_lod` (CDLOD quadtree: patches and morph ranges),
    `trees` (procedural species meshes, planting in tiles), `habitat_mesher` (chunked meshes; the app
    meshes only the glass and end walls, the landscape pass draws the land), `hull_mesh` (the outside, for the
    partner cylinder, and `partnerTransform`), placeholder star field, mesh primitives,
    `settlements` (towns and farms planned on the unrolled floor: streets that follow the river, lots,
    houses, square, bridge, river front, props, town trees, 1 m ground maps; `stampSettlements` marks them
    in the cover map), `buildings` (their meshes and colliders; facades are packed into
    `Vertex::material` and drawn by the shader), `props` (balls, crates, barrels, bales, cafe furniture:
    meshes and collision parts), `clouds` (the cloud map: cover and detail wrapped once round the habitat and
    once along it), `birds` (flocks wheeling over fixed places on the floor, worked out fresh each frame)
  - `audio/`: `Soundscape`, the synthesised ambience (wind, leaves, water, rain, birds or crickets, a town's
    murmur, footsteps). It fills a buffer of frames and knows nothing about devices, so it is unit tested
  - The terrain and settlement sources are compiled with `-O2` even in Debug (`src/core/CMakeLists.txt`),
    or world generation takes several seconds
  - `scenario/`: TOML habitat files (toml++, used only in `scenario.cpp`; bump `kGeneratorVersion` when
    generation changes; `[climate]` holds the cloud deck, how often it rains and the length of the year,
    `season_at_epoch` deciding where in that year J2000 falls, and so which season a preset starts in);
    `gpu_abi/`: uniform structs, SPIR-V reflection, `color_grade` (the grading LUT),
    `ground_atlas` (the towns' ground maps packed for the landscape shader); plus camera, frustum, sim
    clock, app options
- `physics` (`StarshipSimulator_physics`): `PhysicsWorld`, Jolt Physics v5.6.0 in double precision behind a
  pimpl (Jolt is linked PRIVATE: its target exports `-mavx2` and its config defines). Jolt's gravity is
  zero: the `SpinFrame` step listener gives moving bodies the centrifugal kick and turns their velocity for
  Coriolis (leapfrog: launched bodies get a half kick), plus buoyancy in water. Terrain collision is
  built from the terrain grid in tiles (the drawn triangles) around the player and moving props, tree
  trunks per tree tile near the player; the player is a `CharacterVirtual` stood along the local up
  every step. No SDL or render includes (layering test). Include Jolt only through `src/physics/jolt.h`
  (Jolt.h must come first). Jolt is built with `-O2` even in Debug. Unit tests link core and physics
- `audio` (`StarshipSimulator_audio`): `AudioDevice`, SDL's audio device pulling frames from `core/audio`'s
  `Soundscape` on its own thread (a mutex guards it). No renderer, ImGui or Jolt includes (layering test); a
  machine without sound just stays quiet
- `render` (`StarshipSimulator_render`): SDL_GPU device, shader library, `GpuWorld` (all habitat chunks in one
  vertex/index buffer), `GpuLandscape` (height, cover and profile textures, the instanced CDLOD patch mesh),
  `GpuTrees` (instances by tile, two detail levels), `ShadowMap` (trees, along the dominant mirror beam),
  `DynamicBuffer` (per-frame storage uploads), `GpuSettlements` (town meshes, ground-map atlas), `GpuProps`
  (prop meshes, per-frame instances), `GpuBirds` (this frame's birds, no mesh: six vertices each in the
  shader), passes (Milky Way, stars, planets, Earth/Moon, partner hull, terrain disks, landscape, trees,
  buildings, props, mirrors, birds, markers, water, glass, clouds, rain, tonemap with the grade LUT; drawn in
  that order after the shadow pass, which holds trees, buildings and props), `texture.h` (2D uploads with
  GPU mipmaps), `pipeline.h` helper, ImGui layer
  - The cloud deck is ray-marched, so it is drawn on a stand-in cylinder just outside the deck's base
    (`cloud_shell.vert`) rather than on a full-screen triangle, and writes no depth: the depth test then
    throws away every pixel with land in front before the march runs, and `layout(early_fragment_tests)`
    makes sure it happens early. From below the deck the pass draws the faces where rays enter that cylinder,
    from inside it the faces where they leave, so each pixel is shaded exactly once
- `src/app/`: the `StarshipSimulator` executable (main loop, input, HUD, sky data loading on a worker thread);
  its headers are private. The weather and the season come from `weatherAt` each frame; the HUD's sliders
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
  lookup, `include/terrain_colors.glsl` the fields, meadows, shore and town colours, `include/lit.glsl` the
  light on buildings and props (hill and shadow-map shadows, lights at night), `include/town_ground.glsl`
  the ground-map lookup (take screen derivatives before calling: it samples in divergent control flow)
- `data/presets/*.toml`: scenario presets, copied to `build/<preset>/bin/data` at build time
- `tests/`: GoogleTest files, named `*_test.cpp`, all in `StarshipSimulator_tests`
- `cmake/ProjectOptions.cmake`: `StarshipSimulator_configure_target()` (warnings, sanitizers, coverage, tidy)
- `cmake/Dependencies.cmake`: third-party libraries via FetchContent (installed packages win)

## Conventions
- Headers are `.h` (never `.hpp`) and use `#pragma once`
- Code lives in `namespace StarshipSimulator`; project includes use quotes: `#include "StarshipSimulator/core/camera.h"`
- Every new target of ours must call `StarshipSimulator_configure_target(<target>)`; third-party targets must not
- New source files go into the relevant `CMakeLists.txt`; new tests go into `tests/CMakeLists.txt`; new shaders
  into `shaders/CMakeLists.txt`
- Warnings are part of the build: code must compile cleanly with `-Werror` under both GCC and Clang
- Math: use the aliases in `core/math.h` (`Vec3d`, `Mat4f`, ...), never raw `glm::` types. GLM is configured
  with `GLM_FORCE_EXPLICIT_CTOR`, so double → float conversions must be explicit
- Habitat frame: spin axis +Z, the Sun toward +Z; window i is centred on angle i * 2pi/strips, land strip i halfway
  to the next window; "up" is toward the axis. Everything is simulated in this rotating frame
- Sky frame: J2000 equatorial (EQJ). `astro::habitatFromEqj(sunDirection, spinPhase)` turns it into the habitat
  frame; shaders get only that matrix (never times). Simulated time (`SimTime`) is separate from the spin, which
  always runs in real time
- Determinism: procedural generation uses our own `SplitMix64`/`SimplexNoise` (never `std::` distributions) and
  core builds with `-ffp-contract=off`; `buildHabitatMeshes` output must not depend on the thread count
- Precision: world positions are `double`. The GPU only sees camera-relative `float` data (compute
  `position - camera` in double, then convert). Depth is reverse-Z with an infinite far plane: clear to 0,
  compare `GREATER`
- Logging: `core/log.h` (`log::info/warn/error`, std::format). No printf-style varargs anywhere: for ImGui text
  use `ui::text/field/textWrapped(std::format(...))` from `render/imgui_layer.h`, never `ImGui::Text("%...")`
  (the one exception is Jolt's `Trace` callback, which only logs its format string)
- `SDL_Event` is a union: read it only inside `SdlInput::handleEvent` (`src/app/sdl_input.cpp`, NOLINT region)
- SDL_GPU create-info structs use designated initializers naming only the fields that matter (the render target
  disables the missing-field warning for this)

## SDL_GPU shader rules (Vulkan / SPIR-V)
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
