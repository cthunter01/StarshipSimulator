#include "StarshipSimulator/audio/AudioDevice.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include <SDL3/SDL_audio.h>
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_init.h>

#include "StarshipSimulator/core/audio/Soundscape.h"
#include "StarshipSimulator/core/log.h"

namespace StarshipSimulator
{

namespace
{
constexpr int kSampleRate = 48000;
}

struct AudioDevice::Impl
{
    explicit Impl(std::uint64_t seed) : soundscape(kSampleRate, seed) { }

    /// SDL asks for more frames on its audio thread.
    static void SDLCALL feed(void* user, SDL_AudioStream* stream, int additional, int /*total*/)
    {
        auto& self = *static_cast<Impl*>(user);
        if (additional <= 0)
        {
            return;
        }
        const auto count = static_cast<std::size_t>(additional) / sizeof(float);
        self.buffer.resize(count - (count % 2));
        {
            const std::scoped_lock lock(self.mutex);
            self.soundscape.render(self.buffer);
        }
        SDL_PutAudioStreamData(stream, self.buffer.data(),
                               static_cast<int>(self.buffer.size() * sizeof(float)));
    }

    std::mutex         mutex;
    audio::Soundscape  soundscape;
    std::vector<float> buffer;
    SDL_AudioStream*   stream = nullptr;
};

AudioDevice::AudioDevice(std::uint64_t seed) : impl_(std::make_unique<Impl>(seed))
{
    if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
    {
        log::warn("No sound: {}", SDL_GetError());
        return;
    }
    const SDL_AudioSpec spec{.format = SDL_AUDIO_F32, .channels = 2, .freq = kSampleRate};
    impl_->stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, &Impl::feed,
                                              impl_.get());
    if (impl_->stream == nullptr)
    {
        log::warn("No sound: {}", SDL_GetError());
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return;
    }
    SDL_ResumeAudioStreamDevice(impl_->stream);
    log::info("Sound: {} Hz stereo on {}", kSampleRate,
              SDL_GetAudioDeviceName(SDL_GetAudioStreamDevice(impl_->stream)));
}

AudioDevice::~AudioDevice()
{
    if (impl_->stream != nullptr)
    {
        SDL_DestroyAudioStream(impl_->stream);  // stops the callback before the soundscape goes
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
    }
}

void AudioDevice::setMix(const audio::SoundMix& mix)
{
    if (impl_->stream != nullptr)
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->soundscape.setMix(mix);
    }
}

void AudioDevice::play(audio::Sound sound, double level)
{
    if (impl_->stream != nullptr)
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->soundscape.play(sound, level);
    }
}

bool AudioDevice::isOpen() const
{
    return impl_->stream != nullptr;
}

}  // namespace StarshipSimulator
