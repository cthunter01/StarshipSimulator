#include "StarshipSimulator/core/parse_number.h"

#include <gtest/gtest.h>

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

TEST(ParseNumber, ReadsWholeIntegers)
{
    EXPECT_EQ(parseInt("1920"), 1920);
    EXPECT_EQ(parseInt("-14"), -14);
    EXPECT_EQ(parseInt("0"), 0);
}

TEST(ParseNumber, ReadsDecimalsInAnyLocale)
{
    // The decimal point is always '.': habitat files and catalogs are written that way everywhere.
    EXPECT_EQ(parseDouble("2.5"), 2.5);
    EXPECT_EQ(parseDouble("-0.125"), -0.125);
    EXPECT_EQ(parseDouble("45"), 45.0);
    EXPECT_EQ(parseDouble("1e3"), 1000.0);
    EXPECT_EQ(parseDouble("6.02e-3"), 6.02e-3);
}

TEST(ParseNumber, RefusesAnythingThatIsNotAllNumber)
{
    for (const char* text : {"", " 1", "1 ", "1x", "x", "1,5", "+1", "--1"})
    {
        EXPECT_FALSE(parseDouble(text).has_value()) << '"' << text << '"';
        EXPECT_FALSE(parseInt(text).has_value()) << '"' << text << '"';
    }
    EXPECT_FALSE(parseInt("2.5").has_value());
    EXPECT_FALSE(parseInt("99999999999").has_value());  // does not fit in an int
    EXPECT_FALSE(parseDouble("1e999").has_value());     // does not fit in a double
}

}  // namespace
