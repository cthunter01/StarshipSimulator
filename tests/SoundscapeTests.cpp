#include "StarshipSimulator/core/audio/Soundscape.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

#include <gtest/gtest.h>

namespace
{

using StarshipSimulator::audio::Sound;
using StarshipSimulator::audio::SoundMix;
using StarshipSimulator::audio::Soundscape;

constexpr int kRate = 48000;

/// Renders `seconds` of sound and returns it (interleaved stereo).
std::vector<float> listen(Soundscape& soundscape, double seconds)
{
    std::vector<float> samples(static_cast<std::size_t>(seconds * kRate) * 2);
    // In device-sized chunks, as SDL asks for them.
    for (std::size_t at = 0; at < samples.size(); at += 1024)
    {
        const std::size_t count = std::min<std::size_t>(1024, samples.size() - at);
        soundscape.render(std::span(samples).subspan(at, count));
    }
    return samples;
}

double rms(const std::vector<float>& samples, std::size_t from = 0)
{
    double sum = 0.0;
    for (std::size_t i = from; i < samples.size(); ++i)
    {
        sum += static_cast<double>(samples[i]) * static_cast<double>(samples[i]);
    }
    return std::sqrt(sum / static_cast<double>(samples.size() - from));
}

double peak(const std::vector<float>& samples)
{
    float most = 0.0F;
    for (const float s : samples)
    {
        most = std::max(most, std::abs(s));
    }
    return static_cast<double>(most);
}

TEST(Soundscape, IsSilentWithNothingToPlay)
{
    Soundscape quiet(kRate, 1);
    quiet.setMix(SoundMix{});
    const std::vector<float> samples = listen(quiet, 0.5);
    EXPECT_LT(peak(samples), 1e-6);
}

TEST(Soundscape, EachBedIsAudibleAndNoneClips)
{
    const auto level = [](SoundMix mix) {
        Soundscape soundscape(kRate, 7);
        soundscape.setMix(mix);
        const std::vector<float> samples = listen(soundscape, 2.0);
        EXPECT_LT(peak(samples), 1.0);  // the limiter keeps it in range
        for (const float s : samples)
        {
            EXPECT_TRUE(std::isfinite(s));
            if (!std::isfinite(s))
            {
                break;
            }
        }
        return rms(samples, samples.size() / 4);  // after the glide in
    };
    const double wind   = level({.wind = 1.0, .windSpeedMS = 6.0});
    const double leaves = level({.leaves = 1.0});
    const double water  = level({.water = 1.0});
    const double rain   = level({.rain = 1.0});
    const double town   = level({.town = 1.0});
    for (const double bed : {wind, leaves, water, rain, town})
    {
        EXPECT_GT(bed, 0.005);  // audible
        EXPECT_LT(bed, 0.5);    // and not deafening
    }
    // Everything at once stays under full scale.
    const double all = level({.wind        = 1.0,
                              .windSpeedMS = 15.0,
                              .leaves      = 1.0,
                              .water       = 1.0,
                              .rain        = 1.0,
                              .town        = 1.0,
                              .birds       = 1.0});
    EXPECT_LT(all, 0.8);
}

TEST(Soundscape, GlidesToANewMixWithoutAJump)
{
    Soundscape soundscape(kRate, 3);
    soundscape.setMix({.water = 1.0});
    const std::vector<float> samples = listen(soundscape, 0.3);
    // The first milliseconds are much quieter than a moment later: it fades in.
    const std::vector<float> start(samples.begin(), samples.begin() + 96);  // 1 ms
    const std::vector<float> later(samples.end() - 9600, samples.end());    // the last 100 ms
    EXPECT_LT(rms(start), 0.25 * rms(later));
}

TEST(Soundscape, IsTheSameEveryTime)
{
    Soundscape     a(kRate, 11);
    Soundscape     b(kRate, 11);
    const SoundMix mix{.wind = 0.7, .windSpeedMS = 4.0, .water = 0.4, .birds = 1.0};
    a.setMix(mix);
    b.setMix(mix);
    EXPECT_EQ(listen(a, 0.5), listen(b, 0.5));
}

TEST(Soundscape, BirdsSingOnlyWhenThereAreBirds)
{
    Soundscape birds(kRate, 5);
    birds.setMix({.birds = 1.0});
    const std::vector<float> song = listen(birds, 4.0);
    EXPECT_GT(peak(song), 0.01);

    Soundscape none(kRate, 5);
    none.setMix({.birds = 0.0});
    EXPECT_LT(peak(listen(none, 4.0)), 1e-6);
}

TEST(Soundscape, AFootstepIsShortAndSharp)
{
    Soundscape steps(kRate, 9);
    steps.setMix(SoundMix{});
    steps.play(Sound::Footstep);
    const std::vector<float> samples = listen(steps, 0.3);
    const std::vector<float> knock(samples.begin(), samples.begin() + 4800);  // the first 50 ms
    const std::vector<float> after(samples.begin() + 19200, samples.end());   // after 200 ms
    EXPECT_GT(peak(knock), 0.02);
    EXPECT_LT(peak(after), 1e-6);  // gone
}

TEST(Soundscape, TheVolumeSliderScalesEverything)
{
    const auto loudness = [](double master) {
        Soundscape soundscape(kRate, 13);
        soundscape.setMix({.wind = 0.6, .windSpeedMS = 3.0, .master = master});
        const std::vector<float> samples = listen(soundscape, 1.0);
        return rms(samples, samples.size() / 2);
    };
    EXPECT_NEAR(loudness(0.5) / loudness(1.0), 0.5, 0.08);
    EXPECT_LT(loudness(0.0), 0.01 * loudness(1.0));  // faded out to below -40 dB
}

}  // namespace
