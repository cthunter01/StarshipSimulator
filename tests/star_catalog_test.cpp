#include "StarshipSimulator/core/astro/star_catalog.h"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/star_field.h"

namespace StarshipSimulator::astro
{
namespace
{

// A few rows of the HYG v4.4 CSV (some columns shortened), including Sol and a star with no names.
constexpr std::string_view kHygSample =
    R"("id","hip","hd","hr","gl","bf","proper","ra","dec","dist","pmra","pmdec","rv","mag","absmag","spect","ci","x","y","z","vx","vy","vz","rarad","decrad","pmrarad","pmdecrad","bayer","flam","con","comp","comp_primary","base","lum","var","var_min","var_max"
0,,,,"","",Sol,0.0,0.0,0.0,0.0,0.0,0.0,-26.7,4.85,G2V,0.656,0.000005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"","","",1,0,"",1.0,"",,
27919,27989,39801,2061,"","58Alp Ori",Betelgeuse,5.919529,7.407063,152.6718,27.33,10.86,21.0,0.45,-5.469,M2Ib,1.5,3.189296,151.364387,19.682142,-0.0000198,0.00002068,0.00001074,1.5497291183713153,0.12927763169419373,0.0000001324995789,0.000000052650765,Alp,"58",Ori,1,27919,"",13415.28798266127,Alp,0.571,0.281
32263,32349,48915,2491,Gl 244A,"9Alp CMa",Sirius,6.752481,-16.716116,2.6371,-546.01,-1223.08,-9.4,-1.44,1.454,A0m...,0.009,-0.494323,2.476731,-0.758485,0.00000953,-0.00001207,-0.00001221,1.7677953696021995,-0.291751258517685,-0.0000026471311772,-0.000005929659164,Alp,"9",CMa,1,32263,Gl 244,22.824433121735034,"",-1.333,-1.523
104046,104382,177482,7228,"",Sig Oct,"Polaris Australis",21.146119,-88.956499,86.1326,25.96,5.02,12.0,5.45,0.774,F0III,0.283,1.150845,-1.065873,-86.118423,0.00000907,0.00000638,-0.00001223,5.536041063254276,-1.5525837960455975,0.0000001258576315,0.000000024337646,Sig,"",Oct,1,104046,"",42.69725940480335,Sig,5.483,5.423
104214,104217,201091,8085,Gl 820A,"61    Cyg",,21.115055,38.749415,3.4964,4105.76,3155.94,-65.7,5.2,7.487,K5V,1.069,1.4351,-1.320483,2.192451,0.00006032,0.00000745,0.00007151,5.528035829024226,0.6763042474917962,0.0000199054346458,0.000015300322318,"","61",Cyg,1,104214,Gl 820,0.08790225168308823,"",,
3,3,224699,,"","","",0.000335,38.859279,442.4779,5.24,-2.91,0.0,6.61,-1.619,B9,-0.019,344.552785,0.030213,277.614965,0.00000392,0.00001124,-0.00000486,0.0000876262870725,0.6782223625543176,0.0000000254042369,-0.000000014108078,"","",And,1,3,"",386.9011316551087,"",,
99999,,,,"","","",12.0,10.0,100000.0,0.0,0.0,0.0,6.9,0.0,,,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"","","",1,99999,"",1.0,"",,
)";

Vec3d fromRaDec(double raHours, double decDegrees)
{
    const double ra  = raHours * (kPi / 12.0);
    const double dec = degreesToRadians(decDegrees);
    return {std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec)};
}

TEST(StarCatalog, ParsesHygRows)
{
    const auto catalog = parseHygCatalog(kHygSample, 7.0);
    ASSERT_TRUE(catalog.has_value()) << catalog.error();
    const auto& stars = catalog->stars;
    ASSERT_EQ(stars.size(), 6U);  // not Sol

    // Brightest first.
    EXPECT_EQ(stars[0].properName, "Sirius");
    EXPECT_EQ(stars[0].designation, "Alpha Canis Majoris");
    EXPECT_EQ(stars[0].constellation, "CMa");
    EXPECT_EQ(stars[0].hip, 32349);
    EXPECT_DOUBLE_EQ(stars[0].magnitude, -1.44);
    EXPECT_NEAR(stars[0].distanceParsecs, 2.6371, 1e-9);
    EXPECT_LT(glm::length(stars[0].direction - fromRaDec(6.752481, -16.716116)), 1e-12);

    EXPECT_EQ(stars[1].properName, "Betelgeuse");
    EXPECT_EQ(stars[1].designation, "Alpha Orionis");
    EXPECT_DOUBLE_EQ(stars[1].colorIndex, 1.5);

    EXPECT_EQ(stars[2].designation, "61 Cygni");  // Flamsteed number when there is no Bayer letter
    EXPECT_EQ(displayName(stars[2]), "61 Cygni");
    EXPECT_EQ(stars[3].properName, "Polaris Australis");
    EXPECT_EQ(stars[3].designation, "Sigma Octantis");
    EXPECT_EQ(stars[4].designation, "HIP 3");
    EXPECT_EQ(stars[5].designation, "HYG 99999");
    EXPECT_DOUBLE_EQ(stars[5].distanceParsecs, 0.0);  // HYG's 100000 pc means "unknown"
}

TEST(StarCatalog, AppliesTheMagnitudeLimit)
{
    const auto catalog = parseHygCatalog(kHygSample, 1.0);
    ASSERT_TRUE(catalog.has_value());
    ASSERT_EQ(catalog->stars.size(), 2U);
    EXPECT_EQ(catalog->stars[1].properName, "Betelgeuse");
}

TEST(StarCatalog, RejectsMissingColumns)
{
    const auto catalog = parseHygCatalog("\"id\",\"ra\",\"dec\"\n1,2.0,3.0\n", 7.0);
    ASSERT_FALSE(catalog.has_value());
    EXPECT_NE(catalog.error().find("missing column 'hip'"), std::string::npos);
}

TEST(StarCatalog, RejectsShortRows)
{
    std::string csv(kHygSample.substr(0, kHygSample.find('\n') + 1));
    csv += "1,2,3\n";
    EXPECT_FALSE(parseHygCatalog(csv, 7.0).has_value());
}

TEST(StarCatalog, IdentifiesTheStarUnderTheCrosshair)
{
    const auto catalog = parseHygCatalog(kHygSample, 7.0).value();
    const auto sirius  = fromRaDec(6.752481, -16.716116);

    // Sirius is the brightest, so the first star in the catalog.
    EXPECT_EQ(catalog.identify(sirius, degreesToRadians(3.0)), 0U);
    EXPECT_EQ(catalog.stars.front().properName, "Sirius");

    // Slightly off: still Sirius. Far from every star: nothing.
    EXPECT_EQ(catalog.identify(fromRaDec(6.80, -17.5), degreesToRadians(3.0)), 0U);
    EXPECT_FALSE(catalog.identify(fromRaDec(0.0, -40.0), degreesToRadians(3.0)).has_value());
}

TEST(StarCatalog, PrefersABrightStarOverAFaintOneAtSimilarDistance)
{
    const auto star = [](const Vec3d& direction, double magnitude) {
        CatalogStar result;
        result.direction = direction;
        result.magnitude = magnitude;
        return result;
    };
    StarCatalog catalog;
    catalog.stars.push_back(star(fromRaDec(1.0, 0.0), 1.0));
    catalog.stars.push_back(star(fromRaDec(1.0, 1.2), 6.5));
    // The crosshair is a little closer to the faint star, but the bright one is what people mean.
    EXPECT_EQ(catalog.identify(fromRaDec(1.0, 0.7), degreesToRadians(3.0)), 0U);
    // Right on the faint star, it wins.
    EXPECT_EQ(catalog.identify(fromRaDec(1.0, 1.2), degreesToRadians(3.0)), 1U);
}

TEST(StarCatalog, FindsStarsByName)
{
    const auto catalog = parseHygCatalog(kHygSample, 7.0).value();
    ASSERT_NE(catalog.find("sirius"), nullptr);
    EXPECT_EQ(catalog.find("SIRIUS")->hip, 32349);
    ASSERT_NE(catalog.find("alpha orionis"), nullptr);
    EXPECT_EQ(catalog.find("alpha orionis")->properName, "Betelgeuse");
    EXPECT_EQ(catalog.find("Vulcan"), nullptr);
    EXPECT_EQ(catalog.find(""), nullptr);
}

TEST(StarCatalog, ColourIndexGivesPlausibleTemperatures)
{
    EXPECT_NEAR(colorIndexToKelvin(0.656), 5800.0, 150.0);  // the Sun
    EXPECT_NEAR(colorIndexToKelvin(0.0), 10000.0, 500.0);   // Vega-like
    EXPECT_NEAR(colorIndexToKelvin(1.5), 3600.0, 250.0);    // Betelgeuse
    EXPECT_GT(colorIndexToKelvin(-0.3), colorIndexToKelvin(0.0));
}

TEST(StarCatalog, GpuStarsGetBrighterWithLowerMagnitude)
{
    const GpuStar bright = gpuStar(Vec3d(0.0, 0.0, 2.0), -1.0, 6000.0);
    const GpuStar faint  = gpuStar(Vec3d(0.0, 0.0, 2.0), 4.0, 6000.0);
    EXPECT_NEAR(glm::length(Vec3f(bright.direction)), 1.0F, 1e-6F);
    EXPECT_GT(bright.direction.w, faint.direction.w);  // sprite size
    // Five magnitudes are a factor of 100 in flux.
    EXPECT_NEAR(bright.color.y / faint.color.y, 100.0F, 0.01F);
}

TEST(StarCatalog, ConstellationNames)
{
    EXPECT_EQ(constellationName("Ori"), "Orion");
    EXPECT_EQ(constellationGenitive("CVn"), "Canum Venaticorum");
    EXPECT_EQ(constellationName("Xyz"), "");
}

// ---- Naming what is in the sky ------------------------------------------------------------------

TEST(SkyObjects, PhasesFromGeometry)
{
    VisibleBody body;
    body.direction = Vec3d(1.0, 0.0, 0.0);
    body.towardSun = Vec3d(-1.0, 0.0, 0.0);  // the Sun is behind us: full
    EXPECT_NEAR(illuminatedFraction(body), 1.0, 1e-12);
    EXPECT_STREQ(phaseName(illuminatedFraction(body)), "full");
    body.towardSun = Vec3d(1.0, 0.0, 0.0);  // behind the body: new
    EXPECT_NEAR(illuminatedFraction(body), 0.0, 1e-12);
    body.towardSun = Vec3d(0.0, 1.0, 0.0);  // from the side: half
    EXPECT_NEAR(illuminatedFraction(body), 0.5, 1e-12);
    EXPECT_STREQ(phaseName(0.2), "crescent");
    EXPECT_STREQ(phaseName(0.8), "gibbous");
}

TEST(SkyObjects, IdentifiesEarthPlanetsAndStars)
{
    const SkyState sky =
        computeSky(Location::EarthMoonL5, parseIsoTime("2045-06-15T09:00:00Z").value());
    const auto catalog = parseHygCatalog(kHygSample, 7.0).value();
    for (const VisibleBody& body : sky.bodies)
    {
        if (body.body == Body::Earth || body.body == Body::Jupiter)
        {
            const auto found = identifyInSky(body.direction, sky, &catalog);
            EXPECT_TRUE(found.has_value());
            EXPECT_EQ(found.value_or(Identified{}).name, bodyName(body.body));
        }
        if (body.body == Body::Earth)
        {
            EXPECT_NE(describeBody(body).find("km away"), std::string::npos);
        }
    }
    const Vec3d      sirius = fromRaDec(6.752481, -16.716116);
    const Identified star   = identifyInSky(sirius, sky, &catalog).value_or(Identified{});
    EXPECT_EQ(star.name, "Sirius");
    EXPECT_NE(star.details.find("Alpha Canis Majoris in Canis Major"), std::string::npos);
    EXPECT_NE(star.details.find("8.6 light years"), std::string::npos);

    // Without the catalog, the star cannot be named.
    EXPECT_NE(identifyInSky(sirius, sky, nullptr).value_or(Identified{}).name, "Sirius");
}

TEST(SkyData, FullHygCatalogLoads)
{
    const auto path = std::filesystem::path(STARSHIPSIMULATOR_SKY_DATA_DIR) / "hyg_v44.csv.gz";
    if (!std::filesystem::exists(path))
    {
        GTEST_SKIP() << "sky data not downloaded";
    }
    const auto catalog = loadHygCatalog(path, 6.5);
    ASSERT_TRUE(catalog.has_value()) << catalog.error();
    EXPECT_GT(catalog->stars.size(), 8000U);  // the naked-eye sky
    EXPECT_LT(catalog->stars.size(), 10000U);
    EXPECT_EQ(catalog->stars.front().properName, "Sirius");
    std::size_t named = 0;
    for (const CatalogStar& star : catalog->stars)
    {
        named += star.properName.empty() ? 0U : 1U;
        EXPECT_FALSE(displayName(star).empty());
    }
    EXPECT_GT(named, 300U);
}

}  // namespace
}  // namespace StarshipSimulator::astro
