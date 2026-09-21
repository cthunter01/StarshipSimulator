#pragma once

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_keycode.h>
#include <SDL3/SDL_video.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/player_controller.h"

namespace StarshipSimulator
{

class ImGuiLayer;

/// Everything the input devices asked for since the last frame.
struct InputFrame
{
    MoveIntent move;
    Vec2d      lookDelta{0.0};          // mouse motion in pixels while the mouse is captured
    double     wheel            = 0.0;  // scroll steps, positive = up/away
    bool       quit             = false;
    bool       toggleLocomotion = false;
    bool       toggleHud        = false;
    bool       reloadShaders    = false;
    bool       screenshot       = false;
    bool       toggleComfort    = false;
    bool       toggleWings      = false;  // strap the wings on, or take them off
    bool       throwBall        = false;
    bool       kick             = false;  // push whatever is in front of you
    bool       toggleEditor     = false;
    bool       identify         = false;  // name what is under the crosshair
    bool       toggleZoom       = false;  // binoculars
    bool       togglePause      = false;  // stop or restart the clock
    int        timeSteps        = 0;      // time scale steps: positive faster, negative slower
};

/// Turns SDL events and keyboard state into an InputFrame, and manages mouse capture: click the
/// view to capture the mouse for mouse-look, Esc to release it. ImGui gets events first and keeps
/// the ones it wants while the mouse is free.
class SdlInput
{
public:
    explicit SdlInput(SDL_Window* window);

    [[nodiscard]] InputFrame poll(ImGuiLayer& imgui);

    [[nodiscard]] bool mouseCaptured() const { return mouseCaptured_; }
    void               setMouseCaptured(bool captured, ImGuiLayer& imgui);

private:
    void handleEvent(const SDL_Event& event, InputFrame& frame, ImGuiLayer& imgui);
    void handleKey(SDL_Keycode key, InputFrame& frame, ImGuiLayer& imgui);

    SDL_Window* window_;
    bool        mouseCaptured_ = false;
};

}  // namespace StarshipSimulator
