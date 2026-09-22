#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <print>
#include <span>
#include <string_view>
#include <vector>

// On Windows this supplies the real entry point, which hands main() its arguments in UTF-8 (the C
// runtime's own would be in the ANSI code page); on Linux and macOS it changes nothing.
#include <SDL3/SDL_main.h>

#include "Application.h"
#include "StarshipSimulator/core/app_options.h"

int main(int argc, char* argv[])
{
    try
    {
        const std::span<char*>        rawArgs(argv, static_cast<std::size_t>(argc));
        std::vector<std::string_view> args;
        for (const char* arg : rawArgs.subspan(std::min<std::size_t>(1, rawArgs.size())))
        {
            args.emplace_back(arg);
        }

        const auto options = StarshipSimulator::parseAppOptions(args);
        if (!options)
        {
            std::println(stderr, "error: {}\n\n{}", options.error(), StarshipSimulator::appUsage());
            return EXIT_FAILURE;
        }
        if (options->showHelp)
        {
            std::print("{}", StarshipSimulator::appUsage());
            return EXIT_SUCCESS;
        }

        StarshipSimulator::Application application(*options);
        return application.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "error: " << e.what() << '\n';
    }
    return EXIT_FAILURE;
}
