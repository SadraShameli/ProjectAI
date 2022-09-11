cmake_minimum_required(VERSION 3.15)
project(ProjectAI VERSION 1.0)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20")

add_subdirectory(Libraries/YDLidar)
add_subdirectory(Source)

target_link_libraries(ProjectAI ydlidar_sdk)