#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "StarshipSimulator/core/rng.h"

// The habitat's soundscape, synthesised rather than sampled: wind over the fields, leaves, the
// river, rain, birds and the murmur of a town, mixed from what is around the listener. Nothing here
// touches SDL or the audio device: it fills a buffer of frames, which makes it testable.
namespace StarshipSimulator::audio
{

/// How much of each sound the listener should be hearing, 0..1, and what colours them.
struct SoundMix
{
    double wind        = 0.0;  // how much wind noise, from the weather and how exposed you are
    double windSpeedMS = 0.0;  // faster wind hisses higher
    double leaves      = 0.0;  // trees close by
    double water       = 0.0;  // a river or a lake close by
    double rain        = 0.0;
    double town        = 0.0;  // the murmur of a town
    double birds       = 0.0;  // birdsong by day, crickets at night
    double night       = 0.0;  // 1 once the mirrors are shut
    double inside      = 0.0;  // muffles everything (unused until there are interiors)
    double master      = 1.0;  // the volume slider
};

/// A short sound, started by something happening.
enum class Sound : std::uint8_t
{
    Footstep,  // walking
    Run,       // running: heavier
    Splash,
    Throw,
    Thud,
};

/// The soundscape. Renders interleaved stereo frames at a fixed sample rate; every sound is made
/// from noise and oscillators, so there are no audio files to ship. Not thread-safe: the audio
/// device serialises calls.
class Soundscape
{
public:
    explicit Soundscape(int sampleRate = 48000, std::uint64_t seed = 1975);

    /// What to play from now on. The renderer glides to it over a few tens of milliseconds, so
    /// walking in and out of a wood does not click.
    void setMix(const SoundMix& mix);

    /// Starts a one-off sound (level scales it, 0..1).
    void play(Sound sound, double level = 1.0);

    /// Fills `samples` with interleaved stereo frames (its size must be even).
    void render(std::span<float> samples);

    [[nodiscard]] int sampleRate() const { return sampleRate_; }

private:
    struct Noise
    {
        SplitMix64 rng;
        double     next() { return rng.uniform(-1.0, 1.0); }  // white, -1..1
    };

    /// A one-pole filter: low-passes, and its complement high-passes.
    struct OnePole
    {
        double value = 0.0;
        double low(double input, double coefficient);
    };

    /// A bird call or a cricket chirp being played now.
    struct Chirp
    {
        double remaining = 0.0;  // seconds
        double length    = 0.0;
        double fromHz    = 0.0;
        double toHz      = 0.0;
        double phase     = 0.0;
        double level     = 0.0;
        double pan       = 0.0;
        int    pulses    = 1;  // crickets: several short bursts
    };

    /// A footstep or other knock being played now.
    struct Oneshot
    {
        double remaining = 0.0;
        double length    = 0.0;
        double level     = 0.0;
        double toneHz    = 0.0;
        double phase     = 0.0;
        double noisiness = 1.0;
    };

    /// The filters of one channel.
    struct Channel
    {
        OnePole windLow;
        OnePole windBody;
        OnePole windRumble;
        OnePole leafLow;
        OnePole leafHigh;
        OnePole waterLow;
        OnePole waterHigh;
        OnePole rainLow;
        OnePole rainHigh;
        OnePole townLow;
        OnePole stepLow;
    };

    void                 glide(double amount);
    [[nodiscard]] double drip(double dt);
    void                 sing(std::array<double, 2>& out, double dt);
    [[nodiscard]] double knocks(double dt);
    void                 startCall(bool cricket);
    [[nodiscard]] double coefficient(double cutoffHz) const;
    [[nodiscard]] double ambience(Channel& channel, double gust, double burble, double murmur);

    int      sampleRate_;
    SoundMix target_;
    SoundMix now_;
    Noise    noise_;

    std::array<Channel, 2> channels_{};
    double                 gustPhase_     = 0.0;
    double                 burblePhase_   = 0.0;
    double                 murmurPhase_   = 0.0;
    double                 untilNextCall_ = 0.5;
    double                 dripLevel_     = 0.0;

    std::array<Chirp, 8>   chirps_{};
    std::array<Oneshot, 8> oneshots_{};
};

}  // namespace StarshipSimulator::audio
