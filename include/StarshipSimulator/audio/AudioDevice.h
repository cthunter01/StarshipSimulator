#pragma once

#include <memory>

#include "StarshipSimulator/core/audio/Soundscape.h"

namespace StarshipSimulator
{

/// Plays a Soundscape through SDL's default audio device. The device pulls frames on its own
/// thread; setMix and play only hand it the next mix under a lock, so they are cheap to call every
/// frame. Without a working audio device (a headless machine, a CI runner) it stays silent and
/// every call does nothing.
class AudioDevice
{
public:
    explicit AudioDevice(std::uint64_t seed);
    ~AudioDevice();
    AudioDevice(const AudioDevice&)            = delete;
    AudioDevice& operator=(const AudioDevice&) = delete;
    AudioDevice(AudioDevice&&)                 = delete;
    AudioDevice& operator=(AudioDevice&&)      = delete;

    void setMix(const audio::SoundMix& mix);
    void play(audio::Sound sound, double level = 1.0);

    [[nodiscard]] bool isOpen() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace StarshipSimulator
