cmake_minimum_required(VERSION 3.26)
project(hello_fluid_simulator)
project(chapter1)
project(chapter2)

set(TARGETS hello_fluid_simulator chapter1 chapter2)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP)

include_directories(E:/ComputerGraphics/libraries/OpenGL/includes)
include_directories(E:/ComputerGraphics/libraries/Utilities/includes)
include_directories(E:/ComputerGraphics/libraries/FluidEngine/includes)
include_directories(E:/ComputerGraphics/libraries/Visualime/includes)
link_directories(E:/ComputerGraphics/libraries/OpenGL/lib)
link_directories(E:/ComputerGraphics/libraries/Utilities/lib)
link_directories(E:/ComputerGraphics/libraries/FluidEngine/lib/jet)
link_directories(E:/ComputerGraphics/libraries/Visualime/lib)

add_executable(hello_fluid_simulator src/chapter_1_basic.cpp)
add_executable(chapter1 src/chapter1/animation_fun.cpp)
add_executable(chapter2 src/chapter2/particle_based_simulator.cpp)

foreach (target ${TARGETS})
    target_link_libraries(${target} jet.lib visualime.lib opengl32.lib glfw3.lib)
endforeach ()

if (OPENMP_FOUND)
    foreach (target ${TARGETS})
        target_link_libraries(${target} OpenMP::OpenMP_CXX)
    endforeach ()
endif ()
