cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(DEMO 0 CACHE STRING "Enable demo features (0=off, 1=on)")

# Set compilers to Homebrew's LLVM (which includes OpenMP support)
set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# macOS-specific OpenMP handling
if(APPLE)
   # Manually set OpenMP flaclgs since FindOpenMP might fail
   set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
   set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
   set(OpenMP_C_LIB_NAMES "omp")
   set(OpenMP_CXX_LIB_NAMES "omp")
   set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib")
  
   include_directories("/opt/homebrew/opt/libomp/include")
   link_directories("/opt/homebrew/opt/libomp/lib")
  
   # Still try to find OpenMP for better compatibility
   find_package(OpenMP)
else()
   find_package(OpenMP REQUIRED)
endif()


# Add OpenMP flags if available
if(OpenMP_FOUND)
   message(STATUS "Found OpenMP")
   add_definitions(-DHAVE_OPENMP)
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
   message(WARNING "OpenMP not found - compiling without OpenMP support")
endif()


# Optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG -g")


# Eigen3
find_package(Eigen3 REQUIRED)

add_compile_definitions("DEMO=$<IF:$<BOOL:${DEMO}>,1,0>")

add_executable(RayTracing
   main.cpp
   Camera.hpp
   Object.hpp
   global.hpp
   Triangle.hpp
   Scene.cpp
   Scene.hpp
   Light.hpp
   json.hpp
   AreaLight.hpp
   BVH.cpp
   BVH.hpp
   Bounds3.hpp
   Ray.hpp
   Sphere.hpp
   Material.hpp
   Intersection.hpp
   Renderer.cpp
   Renderer.hpp
   WaveLen.hpp
   lodepng.cpp
   lodepng.h
)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../conf.json
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


target_link_libraries(RayTracing
   PUBLIC
   Eigen3::Eigen
)


# Explicitly link OpenMP if found
if(OpenMP_FOUND)
   target_link_libraries(RayTracing PUBLIC ${OpenMP_CXX_LIBRARIES})
   if(APPLE)
       target_link_libraries(RayTracing PUBLIC "/opt/homebrew/opt/libomp/lib/libomp.dylib")
   endif()
endif()


target_include_directories(RayTracing PRIVATE ${EIGEN3_INCLUDE_DIRS})

