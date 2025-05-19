cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(DEMO 0 CACHE STRING "Enable demo features (0=off, 1=on)")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (DEMO)
    message(STATUS "Demo mode enabled")
    add_definitions(-DDEMO)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEMO")
endif()
# Platform-specific compiler and OpenMP setup
if(APPLE)
    # Use LLVM from Homebrew (supports OpenMP)
    set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
    set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")

    # Manually define OpenMP support
    set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_C_LIB_NAMES "omp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib")

    include_directories("/opt/homebrew/opt/libomp/include")
    link_directories("/opt/homebrew/opt/libomp/lib")

    find_package(OpenMP REQUIRED)
else()
    set(CMAKE_C_COMPILER clang)
    set(CMAKE_CXX_COMPILER clang++)
    find_package(OpenMP REQUIRED)
endif()

# Add OpenMP flags
if(OpenMP_FOUND)
    message(STATUS "Found OpenMP")
    add_definitions(-DHAVE_OPENMP)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS} -L/opt/homebrew/opt/libomp/lib -lomp")
else()
    message(WARNING "OpenMP not found - compiling without OpenMP support")
endif()


# Compilation flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG -g")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Sources
set(SOURCES
    main.cpp Camera.hpp Object.hpp global.hpp Triangle.hpp Scene.cpp
    Scene.hpp Light.hpp json.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Sphere.hpp Material.hpp Intersection.hpp
    Renderer.cpp Renderer.hpp WaveLen.hpp lodepng.cpp lodepng.h
)
list(TRANSFORM SOURCES PREPEND "src/")
add_executable(RayTracing ${SOURCES})

# Copy configuration file
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/conf.json
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
