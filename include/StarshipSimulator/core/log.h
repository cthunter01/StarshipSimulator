#pragma once

#include <cstdio>
#include <format>
#include <print>
#include <utility>

// Minimal logging to stderr. Uses std::format, never printf-style varargs.
namespace StarshipSimulator::log
{

template <typename... Args>
void info(std::format_string<Args...> format, Args&&... args)
{
    std::println(stderr, "[info] {}", std::format(format, std::forward<Args>(args)...));
}

template <typename... Args>
void warn(std::format_string<Args...> format, Args&&... args)
{
    std::println(stderr, "[warn] {}", std::format(format, std::forward<Args>(args)...));
}

template <typename... Args>
void error(std::format_string<Args...> format, Args&&... args)
{
    std::println(stderr, "[error] {}", std::format(format, std::forward<Args>(args)...));
}

}  // namespace StarshipSimulator::log
