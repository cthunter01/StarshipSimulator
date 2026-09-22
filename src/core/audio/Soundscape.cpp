#include "StarshipSimulator/core/audio/Soundscape.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator::audio
{

namespace
{

constexpr double kGlideSeconds = 0.08;  // how quickly the mix follows the listener

double glideTowards(double from, double to, double amount)
{
    return from + ((to - from) * amount);
}

/// Levels of the beds against each other: wind and water loudest, the town a murmur.
constexpr double kWindGain  = 0.42;
constexpr double kLeafGain  = 0.30;
constexpr double kWaterGain = 0.42;
constexpr double kRainGain  = 0.36;
constexpr double kTownGain  = 0.20;
constexpr double kCallGain  = 0.2;
constexpr double kDripGain  = 0.35;
constexpr double kStepGain  = 0.55;

}  // namespace

double Soundscape::OnePole::low(double input, double coefficient)
{
    value += coefficient * (input - value);
    return value;
}

Soundscape::Soundscape(int sampleRate, std::uint64_t seed)
  : sampleRate_(std::max(8000, sampleRate)), noise_{SplitMix64(hashSeed(seed, 0xA0D10))}
{
}

void Soundscape::setMix(const SoundMix& mix)
{
    target_ = mix;
}

void Soundscape::play(Sound sound, double level)
{
    auto* const free =
        std::ranges::find_if(oneshots_, [](const Oneshot& o) { return o.remaining <= 0.0; });
    if (free == oneshots_.end())
    {
        return;
    }
    Oneshot shot;
    shot.level = std::clamp(level, 0.0, 1.0);
    switch (sound)
    {
        case Sound::Footstep:
            shot.length    = 0.07;
            shot.toneHz    = 85.0 + (20.0 * noise_.next());
            shot.noisiness = 0.8;
            shot.level *= 0.6;
            break;
        case Sound::Run:
            shot.length    = 0.09;
            shot.toneHz    = 75.0 + (15.0 * noise_.next());
            shot.noisiness = 0.9;
            break;
        case Sound::Splash:
            shot.length    = 0.45;
            shot.toneHz    = 0.0;
            shot.noisiness = 1.0;
            break;
        case Sound::Throw:
            shot.length    = 0.18;
            shot.toneHz    = 0.0;
            shot.noisiness = 1.0;
            shot.level *= 0.35;
            break;
        case Sound::Thud:
            shot.length    = 0.12;
            shot.toneHz    = 60.0;
            shot.noisiness = 0.4;
            break;
    }
    shot.remaining = shot.length;
    *free          = shot;
}

double Soundscape::coefficient(double cutoffHz) const
{
    return 1.0 - std::exp(-2.0 * kPi * cutoffHz / sampleRate_);
}

void Soundscape::startCall(bool cricket)
{
    auto* const free =
        std::ranges::find_if(chirps_, [](const Chirp& c) { return c.remaining <= 0.0; });
    if (free == chirps_.end())
    {
        return;
    }
    Chirp call;
    call.pan = 0.8 * noise_.next();
    if (cricket)
    {
        call.length = 0.25 + (0.1 * noise_.next());
        call.fromHz = 4300.0 + (300.0 * noise_.next());
        call.toHz   = call.fromHz;
        call.pulses = 4 + static_cast<int>(3.0 * (noise_.next() + 1.0));
        call.level  = 0.35 + (0.2 * noise_.next());
    }
    else
    {
        // A whistle: a quick sweep up or down somewhere between 2 and 6 kHz.
        call.length = 0.06 + (0.05 * (noise_.next() + 1.0));
        call.fromHz = 3200.0 + (1400.0 * noise_.next());
        call.toHz   = call.fromHz * (1.0 + (0.45 * noise_.next()));
        call.pulses = 1;
        call.level  = 0.5 + (0.4 * noise_.next());
    }
    call.remaining = call.length;
    *free          = call;
}

double Soundscape::ambience(Channel& channel, double gust, double burble, double murmur)
{
    // Wind: noise pushed through two low-passes whose cutoff rises with the wind and its gusts.
    const double windCut  = 140.0 + (55.0 * now_.windSpeedMS * gust);
    const double windRaw  = channel.windLow.low(noise_.next(), coefficient(windCut));
    const double windBody = channel.windBody.low(windRaw, coefficient(windCut * 1.6));
    // ...less the lowest rumble, which on headphones is felt more than heard.
    const double wind = (windBody - channel.windRumble.low(windBody, coefficient(70.0))) * 8.0;
    // Leaves: the hiss above a couple of kHz, swelling with each gust.
    const double leafIn   = noise_.next();
    const double leafHigh = leafIn - channel.leafHigh.low(leafIn, coefficient(2200.0));
    const double leaves =
        channel.leafLow.low(leafHigh, coefficient(6500.0)) * (0.25 + (gust * gust));
    // Water: a band of noise, burbling.
    const double waterIn   = noise_.next();
    const double waterBand = channel.waterLow.low(waterIn, coefficient(1400.0)) -
                             channel.waterHigh.low(waterIn, coefficient(280.0));
    const double water     = waterBand * burble * 2.2;
    // Rain: the hiss of a million drops.
    const double rainIn   = noise_.next();
    const double rainHigh = rainIn - channel.rainHigh.low(rainIn, coefficient(700.0));
    const double rain     = channel.rainLow.low(rainHigh, coefficient(9000.0));
    // A town: a low murmur.
    const double town = channel.townLow.low(noise_.next(), coefficient(420.0)) * 3.0 * murmur;

    const double dayOnly = 1.0 - (0.6 * now_.night);
    return (wind * kWindGain * now_.wind) + (leaves * kLeafGain * now_.leaves) +
           (water * kWaterGain * now_.water) + (rain * kRainGain * now_.rain) +
           (town * kTownGain * now_.town * dayOnly);
}

void Soundscape::glide(double amount)
{
    now_.wind        = glideTowards(now_.wind, target_.wind, amount);
    now_.windSpeedMS = glideTowards(now_.windSpeedMS, target_.windSpeedMS, amount);
    now_.leaves      = glideTowards(now_.leaves, target_.leaves, amount);
    now_.water       = glideTowards(now_.water, target_.water, amount);
    now_.rain        = glideTowards(now_.rain, target_.rain, amount);
    now_.town        = glideTowards(now_.town, target_.town, amount);
    now_.birds       = glideTowards(now_.birds, target_.birds, amount);
    now_.night       = glideTowards(now_.night, target_.night, amount);
    now_.inside      = glideTowards(now_.inside, target_.inside, amount);
    now_.master      = glideTowards(now_.master, target_.master, amount);
}

double Soundscape::drip(double dt)
{
    // Drops on leaves and stone while it rains: little clicks.
    dripLevel_ *= 0.93;
    if ((noise_.next() * 0.5) + 0.5 < now_.rain * 90.0 * dt)
    {
        dripLevel_ = 0.4 + (0.6 * ((noise_.next() * 0.5) + 0.5));
    }
    return dripLevel_ * noise_.next() * kDripGain * now_.rain;
}

void Soundscape::sing(std::array<double, 2>& out, double dt)
{
    // Birdsong by day, crickets at night: calls started at random, more where there are more.
    untilNextCall_ -= dt;
    if (untilNextCall_ <= 0.0)
    {
        const double busy = std::max(now_.birds, 0.02);
        untilNextCall_    = (0.12 + (1.6 * ((noise_.next() * 0.5) + 0.5))) / busy;
        if (now_.birds > 0.02)
        {
            startCall(now_.night > 0.5);
        }
    }
    for (Chirp& call : chirps_)
    {
        if (call.remaining <= 0.0)
        {
            continue;
        }
        const double t  = 1.0 - (call.remaining / call.length);  // 0..1 through the call
        const double hz = glm::mix(call.fromHz, call.toHz, t * t);
        call.phase      = std::fmod(call.phase + (hz * dt), 1.0);
        double envelope = std::sin(kPi * t);
        envelope *= envelope;
        if (call.pulses > 1 && std::sin(kPi * t * call.pulses) <= 0.0)
        {
            envelope = 0.0;  // between a cricket's pulses
        }
        const double voice = std::sin(2.0 * kPi * call.phase) * envelope * call.level * kCallGain *
                             std::min(1.0, now_.birds * 2.0);
        out[0] += voice * (0.5 - (0.5 * call.pan));
        out[1] += voice * (0.5 + (0.5 * call.pan));
        call.remaining -= dt;
    }
}

double Soundscape::knocks(double dt)
{
    // Footsteps and knocks: a thump and a scuff, dying away fast.
    double sum = 0.0;
    for (Oneshot& shot : oneshots_)
    {
        if (shot.remaining <= 0.0)
        {
            continue;
        }
        const double age      = shot.length - shot.remaining;
        const double envelope = std::exp(-age / (0.22 * shot.length));
        shot.phase            = std::fmod(shot.phase + (shot.toneHz * dt), 1.0);
        const double tone     = std::sin(2.0 * kPi * shot.phase);
        const double grit     = channels_[0].stepLow.low(
            noise_.next(), coefficient(shot.toneHz > 0.0 ? 1800.0 : 4200.0));
        sum += ((grit * 3.0 * shot.noisiness) + (tone * (1.0 - shot.noisiness))) * envelope *
               shot.level * kStepGain;
        shot.remaining -= dt;
    }
    return sum;
}

void Soundscape::render(std::span<float> samples)
{
    const double dt     = 1.0 / sampleRate_;
    const double amount = 1.0 - std::exp(-dt / kGlideSeconds);
    for (std::size_t i = 0; i + 1 < samples.size(); i += 2)
    {
        glide(amount);

        // Slow swells: the wind's gusts, the river's burble, the town's comings and goings.
        gustPhase_        = std::fmod(gustPhase_ + (dt * (0.07 + (0.012 * now_.windSpeedMS))), 1.0);
        burblePhase_      = std::fmod(burblePhase_ + (dt * 2.3), 1.0);
        murmurPhase_      = std::fmod(murmurPhase_ + (dt * 0.11), 1.0);
        const double gust = 0.55 + (0.3 * std::sin(2.0 * kPi * gustPhase_)) +
                            (0.15 * std::sin((2.0 * kPi * 2.7 * gustPhase_) + 1.3));
        const double burble = 0.6 + (0.25 * std::sin(2.0 * kPi * burblePhase_)) +
                              (0.15 * std::sin((2.0 * kPi * 3.1 * burblePhase_) + 0.4));
        const double murmur = 0.75 + (0.25 * std::sin(2.0 * kPi * murmurPhase_));

        std::array<double, 2> out{ambience(channels_[0], gust, burble, murmur),
                                  ambience(channels_[1], gust, burble, murmur)};
        const double          drops = drip(dt);
        out[0] += drops;
        out[1] += drops * 0.6;
        sing(out, dt);
        const double knock = knocks(dt);
        out[0] += knock;
        out[1] += knock;

        // Indoors (later) muffles everything; the volume slider; a gentle limiter.
        for (std::size_t c = 0; c < 2; ++c)
        {
            const double level = out.at(c) * now_.master * (1.0 - (0.7 * now_.inside));
            samples[i + c]     = static_cast<float>(std::tanh(level));
        }
    }
}

}  // namespace StarshipSimulator::audio
