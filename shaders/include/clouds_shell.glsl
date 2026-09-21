// The cloud deck's stand-in cylinder (cloud_shell.vert). Keep SHELL_SEGMENTS in step with
// CloudPass (habitat_passes.cpp), which draws 12 * SHELL_SEGMENTS vertices.
const int SHELL_SEGMENTS = 128;

// Just outside the lowest the deck's base reaches (clouds.glsl lets it sag by a tenth of the deck).
float shellRadius()
{
    return habitat.cloud.y + (0.1 * (habitat.cloud.y - habitat.cloud.x)) + 5.0;
}
