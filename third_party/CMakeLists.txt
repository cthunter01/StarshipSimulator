# Vendored third-party code. These targets deliberately do not call StarshipSimulator_configure_target():
# our warnings and clang-tidy are for our own code. Headers are SYSTEM includes for the same reason.

# Astronomy Engine v2.1.19 (MIT, https://github.com/cosinekitty/astronomy): positions of the Sun, Moon
# and planets, Lagrange points, rotation axes and constellations, accurate to about an arcminute.
add_library(StarshipSimulator_astronomy STATIC astronomy-engine/astronomy.c)
target_include_directories(StarshipSimulator_astronomy SYSTEM PUBLIC astronomy-engine)
find_library(STARSHIPSIMULATOR_LIBM m)
if(STARSHIPSIMULATOR_LIBM)
    target_link_libraries(StarshipSimulator_astronomy PUBLIC ${STARSHIPSIMULATOR_LIBM})
endif()

# stb_image / stb_image_write (public domain or MIT, https://github.com/nothings/stb, commit 2c980bb):
# JPEG/PNG decoding and zlib inflate; zlib/PNG writing for tests.
add_library(StarshipSimulator_stb STATIC stb/stb_impl.c)
target_include_directories(StarshipSimulator_stb SYSTEM PUBLIC stb)
