# Sky data (about 52 MB): the HYG star catalog, NASA's Milky Way map and Earth/Moon textures.
# Downloaded once at configure time into a cache shared by all presets; the executable finds it there
# (or in bin/data/sky after installation). Without it the simulator falls back to placeholder stars and
# plain-coloured Earth and Moon, so builds still work offline. CI skips the download.
#
# Credits (also shown in the app, see data/CREDITS.md):
#   HYG v4.4 (CC BY-SA 4.0): David Nash, astronexus, https://codeberg.org/astronexus/hyg
#   Milky Way: NASA/Goddard Space Flight Center Scientific Visualization Studio; Gaia DR2: ESA/Gaia/DPAC
#   Earth: NASA Visible Earth (Blue Marble; Black Marble 2012). Moon: NASA SVS CGI Moon Kit (LRO LROC)

if(DEFINED ENV{CI})
    set(sky_download_default OFF)
else()
    set(sky_download_default ON)
endif()
option(STARSHIPSIMULATOR_DOWNLOAD_SKY_DATA "Download the star catalog and sky textures (~52 MB)"
       ${sky_download_default})
set(STARSHIPSIMULATOR_SKY_DATA_DIR "${PROJECT_SOURCE_DIR}/build/_downloads/sky"
    CACHE PATH "Where the downloaded sky data is kept (shared by all presets)")

# Downloads url to <sky data dir>/<name> unless a file with the expected SHA-256 is already there.
# Failures are warnings: the app works without the data.
function(StarshipSimulator_download_sky_file name url sha256)
    set(path "${STARSHIPSIMULATOR_SKY_DATA_DIR}/${name}")
    if(EXISTS "${path}")
        file(SHA256 "${path}" actual)
        if(actual STREQUAL sha256)
            return()
        endif()
    endif()
    message(STATUS "Downloading sky data: ${name}")
    file(DOWNLOAD "${url}" "${path}.part" STATUS status TLS_VERIFY ON INACTIVITY_TIMEOUT 60)
    list(GET status 0 code)
    if(code EQUAL 0)
        file(SHA256 "${path}.part" actual)
        if(actual STREQUAL sha256)
            file(RENAME "${path}.part" "${path}")
            return()
        endif()
        set(status "SHA-256 mismatch (got ${actual})")
    endif()
    file(REMOVE "${path}.part")
    message(WARNING "Could not download ${name} (${status}); the sky will use placeholders.")
endfunction()

if(STARSHIPSIMULATOR_DOWNLOAD_SKY_DATA)
    file(MAKE_DIRECTORY "${STARSHIPSIMULATOR_SKY_DATA_DIR}")
    StarshipSimulator_download_sky_file(hyg_v44.csv.gz
        "https://codeberg.org/astronexus/hyg/media/branch/main/data/hyg/CURRENT/hyg_v44.csv.gz"
        00b349893b9a53106dd488d8371e8d2fa586043e500bb3cdb8bff3931682197d)
    StarshipSimulator_download_sky_file(milkyway_2020_4k.exr
        "https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/milkyway_2020_4k.exr"
        2eb802d6e68d170b410f766c7fec07f7518619f6b6708fdc81e9302d93e74fdb)
    StarshipSimulator_download_sky_file(earth_day_2048.jpg
        "https://eoimages.gsfc.nasa.gov/images/imagerecords/57000/57735/land_ocean_ice_cloud_2048.jpg"
        fb67ac030214c1891994c8f976e7f6c9cd5b0f21586aba8567250781a4fe708e)
    StarshipSimulator_download_sky_file(earth_night_3600.jpg
        "https://eoimages.gsfc.nasa.gov/images/imagerecords/79000/79765/dnb_land_ocean_ice.2012.3600x1800.jpg"
        373e5a08c9f378a2ce6320214a613148e4b1e3946b3f39a516c9093b76cb7124)
    StarshipSimulator_download_sky_file(moon_1k.jpg
        "https://svs.gsfc.nasa.gov/vis/a000000/a004700/a004720/lroc_color_poles_1k.jpg"
        b246064f217f8d479df78c49c7c8595a8f5fbda008a72fd539978d2e121e0109)
endif()
