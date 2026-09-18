include(CheckIPOSupported)

if(STARSHIPSIMULATOR_ENABLE_IPO)
    check_ipo_supported(RESULT STARSHIPSIMULATOR_IPO_SUPPORTED OUTPUT ipo_output)
    if(NOT STARSHIPSIMULATOR_IPO_SUPPORTED)
        message(WARNING "STARSHIPSIMULATOR_ENABLE_IPO is ON, but this toolchain cannot do IPO:\n${ipo_output}")
    endif()
endif()

if("thread" IN_LIST STARSHIPSIMULATOR_SANITIZERS AND "address" IN_LIST STARSHIPSIMULATOR_SANITIZERS)
    message(FATAL_ERROR "STARSHIPSIMULATOR_SANITIZERS: 'thread' cannot be combined with 'address'")
endif()

if(STARSHIPSIMULATOR_ENABLE_COVERAGE AND NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "STARSHIPSIMULATOR_ENABLE_COVERAGE uses llvm-cov and needs Clang. Use the coverage preset.")
endif()

if(STARSHIPSIMULATOR_ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_PROGRAM clang-tidy REQUIRED)
    # GCC-only -W flags are unknown to clang-tidy's Clang frontend; don't report them.
    set(STARSHIPSIMULATOR_CLANG_TIDY_COMMAND ${CLANG_TIDY_PROGRAM} --extra-arg=-Wno-unknown-warning-option)
    if(STARSHIPSIMULATOR_WARNINGS_AS_ERRORS)
        list(APPEND STARSHIPSIMULATOR_CLANG_TIDY_COMMAND --warnings-as-errors=*)
    endif()
endif()

# StarshipSimulator_configure_target(<target>)
# Applies warnings, sanitizers, coverage, clang-tidy and IPO to one of *our* targets (never to dependencies).
# Call it for every target you add.
function(StarshipSimulator_configure_target target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wnon-virtual-dtor
            -Wold-style-cast -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wformat=2
            -Wimplicit-fallthrough -Wcast-align
            $<$<CXX_COMPILER_ID:GNU>:-Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wuseless-cast>
            $<$<BOOL:${STARSHIPSIMULATOR_WARNINGS_AS_ERRORS}>:-Werror>)
        # Bounds-checked operator[] etc. in libstdc++; ABI-compatible, unlike _GLIBCXX_DEBUG.
        target_compile_definitions(${target} PRIVATE $<$<CONFIG:Debug>:_GLIBCXX_ASSERTIONS>)

        if(STARSHIPSIMULATOR_SANITIZERS)
            list(JOIN STARSHIPSIMULATOR_SANITIZERS "," sanitizers)
            set(sanitize -fsanitize=${sanitizers})
            if("undefined" IN_LIST STARSHIPSIMULATOR_SANITIZERS)
                list(APPEND sanitize -fno-sanitize-recover=all)   # UB stops the program, so a test fails
            endif()
            target_compile_options(${target} PRIVATE ${sanitize} -fno-omit-frame-pointer)
            target_link_options(${target} PUBLIC ${sanitize})
        endif()
    endif()

    if(STARSHIPSIMULATOR_ENABLE_COVERAGE)
        target_compile_options(${target} PRIVATE -fprofile-instr-generate -fcoverage-mapping)
        target_link_options(${target} PUBLIC -fprofile-instr-generate)
    endif()

    if(STARSHIPSIMULATOR_ENABLE_CLANG_TIDY)
        set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${STARSHIPSIMULATOR_CLANG_TIDY_COMMAND}")
    endif()

    if(STARSHIPSIMULATOR_ENABLE_IPO AND STARSHIPSIMULATOR_IPO_SUPPORTED)
        set_target_properties(${target} PROPERTIES INTERPROCEDURAL_OPTIMIZATION ON)
    endif()
endfunction()
