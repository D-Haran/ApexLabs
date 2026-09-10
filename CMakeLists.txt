cmake_minimum_required(VERSION 3.24)

project(ApexLab VERSION 0.1.0 LANGUAGES CXX)

option(APEXLAB_BUILD_TESTS "Build the ApexLab test suite" ON)
option(APEXLAB_BUILD_BENCHMARKS "Build lightweight benchmarks" ON)
option(APEXLAB_ENABLE_CLANG_TIDY "Run clang-tidy while compiling" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

function(apexlab_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror=return-type)
    endif()
endfunction()

if(APEXLAB_ENABLE_CLANG_TIDY)
    find_program(APEXLAB_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${APEXLAB_CLANG_TIDY_EXE}")
endif()

add_subdirectory(cpp/sim-core)
add_subdirectory(apps/sim-cli)

if(APEXLAB_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(cpp/tests)
endif()

if(APEXLAB_BUILD_BENCHMARKS)
    add_subdirectory(cpp/benchmarks)
endif()

add_subdirectory(apps/sim-server)
