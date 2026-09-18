# StarshipSimulator

A walk-around simulator for space habitats (O'Neill cylinders, Bishop rings, starships) under an accurate sky.
C++23, CMake presets + Ninja, GoogleTest, SDL3 + SDL_GPU (Vulkan), Dear ImGui. Linux, GCC and Clang.
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
Views: valley, lookup, window, endcap, ramp, sunward, axis, overview (or `--camera x,y,z,yaw,pitch` in the
habitat frame; `--scenario data/presets/coriolis_playground.toml` for the small habitat). Write captures to the
scratchpad and inspect them with the Read tool before reporting visual work as done. The window opens briefly
on the user's desktop (Wayland). Add `--no-gpu-debug` for quicker runs.
- Performance: `build/clang-release/bin/StarshipSimulator --size 1920x1080 --no-vsync --no-gpu-debug --benchmark`
  prints average/p99 frame times over a fixed tour (M1 on the RTX A1000: ~5 ms average, ~9 ms p99)
- ASan build of the app: `LSAN_OPTIONS=suppressions=tools/lsan.supp build/asan/bin/StarshipSimulator --no-gpu-debug ...`
  (system libraries such as libdbus leak on purpose; the Vulkan validation layer leaks a few hundred bytes of its
  own bookkeeping, so leave GPU debug off for leak checks)

## Layout
- `include/StarshipSimulator/<module>/`: public headers; `src/<module>/`: sources
- `core` (`StarshipSimulator_core`): everything that can be unit tested without a GPU. **No SDL, ImGui or
  render includes**: the `layering` test (`cmake/CheckLayering.cmake`) fails otherwise. Unit tests link only core
  - `habitat/`: `OneillCylinderSpec` (the shareable description), `metrics` (spin, gravity, air, hull strength),
    `MeridianProfile` (the revolved cross-section), `HabitatGeometry` (regions, terrain, ground queries),
    `mirror_optics` (where the sun appears, day/night)
  - `physics/`: `RotatingFrame` (centrifugal + Coriolis, exact free flight, `stepFreeBody`), `PlayerController`
  - `procgen/`: deterministic noise, `habitat_mesher` (chunked meshes), star field, mesh primitives
  - `scenario/`: TOML habitat files (toml++, used only in `scenario.cpp`); `gpu_abi/`: uniform structs, SPIR-V
    reflection; plus camera, frustum, sim clock, app options
- `render` (`StarshipSimulator_render`): SDL_GPU device, shader library, `GpuWorld` (all habitat chunks in one
  vertex/index buffer), passes (stars, terrain, mirrors, markers, glass, tonemap), `pipeline.h` helper, ImGui layer
- `src/app/`: the `StarshipSimulator` executable (main loop, input, HUD); its headers are private
- `shaders/`: GLSL, compiled by glslc to `build/<preset>/bin/shaders/*.spv` (`cmake/Shaders.cmake`);
  `include/habitat.glsl` holds the shared air (aerial perspective) and sunlight (mirror beams) models
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
- Determinism: procedural generation uses our own `SplitMix64`/`SimplexNoise` (never `std::` distributions) and
  core builds with `-ffp-contract=off`; `buildHabitatMeshes` output must not depend on the thread count
- Precision: world positions are `double`. The GPU only sees camera-relative `float` data (compute
  `position - camera` in double, then convert). Depth is reverse-Z with an infinite far plane: clear to 0,
  compare `GREATER`
- Logging: `core/log.h` (`log::info/warn/error`, std::format). No printf-style varargs anywhere: for ImGui text
  use `ui::text/field/textWrapped(std::format(...))` from `render/imgui_layer.h`, never `ImGui::Text("%...")`
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
- Include order for uniforms: `frame.glsl` (slot 0) then `habitat.glsl` (slot 1); define `UNIFORM_SET` first

## GPU notes (this machine)
- Laptop with an NVIDIA RTX A1000 (4 GB) and Intel Iris Xe; only the NVIDIA GPU has a Vulkan driver installed.
  The device prefers the discrete GPU; the HUD and the startup log show which GPU is used. If the wrong one is
  picked: `VK_LOADER_DRIVERS_SELECT='nvidia*'`
- Vulkan validation runs in Debug builds (`--gpu-debug` / `--no-gpu-debug`) when `vulkan-validation-layers`
  is installed
