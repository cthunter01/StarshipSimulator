#include "sdl_input.h"

#include <cstddef>
#include <span>

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_keyboard.h>
#include <SDL3/SDL_keycode.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_scancode.h>
#include <SDL3/SDL_video.h>

#include "StarshipSimulator/render/imgui_layer.h"

namespace StarshipSimulator
{

namespace
{

double axis(std::span<const bool> keys, SDL_Scancode positive, SDL_Scancode negative)
{
    return (keys[static_cast<std::size_t>(positive)] ? 1.0 : 0.0) -
           (keys[static_cast<std::size_t>(negative)] ? 1.0 : 0.0);
}

}  // namespace

SdlInput::SdlInput(SDL_Window* window) : window_(window) { }

void SdlInput::setMouseCaptured(bool captured, ImGuiLayer& imgui)
{
    mouseCaptured_ = captured;
    SDL_SetWindowRelativeMouseMode(window_, captured);
    imgui.setMouseEnabled(!captured);
}

void SdlInput::handleKey(SDL_Keycode key, InputFrame& frame, ImGuiLayer& imgui)
{
    switch (key)
    {
        case SDLK_ESCAPE:
            setMouseCaptured(false, imgui);
            break;
        case SDLK_F:
            frame.toggleLocomotion = true;
            break;
        case SDLK_F1:
            frame.toggleHud = true;
            break;
        case SDLK_F5:
            frame.reloadShaders = true;
            break;
        case SDLK_F12:
            frame.screenshot = true;
            break;
        case SDLK_C:
            frame.toggleComfort = true;
            break;
        case SDLK_G:
            frame.throwBall = true;
            break;
        case SDLK_E:
            frame.kick = true;
            break;
        case SDLK_TAB:
            frame.toggleEditor = true;
            break;
        case SDLK_I:
            frame.identify = true;
            break;
        case SDLK_B:
            frame.toggleZoom = true;
            break;
        case SDLK_P:
            frame.togglePause = true;
            break;
        case SDLK_COMMA:
            frame.timeSteps -= 1;
            break;
        case SDLK_PERIOD:
            frame.timeSteps += 1;
            break;
        default:
            break;
    }
}

// SDL_Event is a union discriminated by 'type'; this is the one place we read it.
// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
void SdlInput::handleEvent(const SDL_Event& event, InputFrame& frame, ImGuiLayer& imgui)
{
    const bool keyboardFree = mouseCaptured_ || !imgui.wantsKeyboard();
    const bool mouseFree    = mouseCaptured_ || !imgui.wantsMouse();
    switch (event.type)
    {
        case SDL_EVENT_QUIT:
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            frame.quit = true;
            break;
        case SDL_EVENT_KEY_DOWN:
            if (keyboardFree && !event.key.repeat)
            {
                handleKey(event.key.key, frame, imgui);
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (!mouseCaptured_ && mouseFree && event.button.button == SDL_BUTTON_LEFT)
            {
                setMouseCaptured(true, imgui);
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (mouseCaptured_)
            {
                frame.lookDelta += Vec2d(event.motion.xrel, event.motion.yrel);
            }
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            if (mouseFree)
            {
                frame.wheel += static_cast<double>(event.wheel.y);
            }
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            if (mouseCaptured_)
            {
                setMouseCaptured(false, imgui);
            }
            break;
        default:
            break;
    }
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access)

InputFrame SdlInput::poll(ImGuiLayer& imgui)
{
    InputFrame frame;
    SDL_Event  event;
    while (SDL_PollEvent(&event))
    {
        imgui.processEvent(event);
        handleEvent(event, frame, imgui);
    }

    int                         keyCount = 0;
    const bool*                 state    = SDL_GetKeyboardState(&keyCount);
    const std::span<const bool> keys(state, static_cast<std::size_t>(keyCount));
    if (mouseCaptured_ || !imgui.wantsKeyboard())
    {
        frame.move.forward = axis(keys, SDL_SCANCODE_W, SDL_SCANCODE_S);
        frame.move.right   = axis(keys, SDL_SCANCODE_D, SDL_SCANCODE_A);
        frame.move.up      = axis(keys, SDL_SCANCODE_SPACE, SDL_SCANCODE_LCTRL);
        frame.move.jump    = keys[SDL_SCANCODE_SPACE];
        frame.move.fast    = keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT];
    }
    return frame;
}

}  // namespace StarshipSimulator
