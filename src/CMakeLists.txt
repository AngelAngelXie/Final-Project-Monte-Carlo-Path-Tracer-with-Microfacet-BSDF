cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)
# For debugging
# set(CMAKE_CXX_FLAGS "-DDEBUG")

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(RayTracing main.cpp Camera.hpp Object.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp json.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Sphere.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp WaveLen.hpp)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../conf.json
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(RayTracing PUBLIC OpenMP::OpenMP_CXX)
