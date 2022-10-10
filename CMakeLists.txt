cmake_minimum_required(VERSION 3.15)

# Project Specification
project(ProjectAI VERSION 1.0)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

# Setting C++ Standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20")

# Main Source Files
add_subdirectory(Source)

# YDLidar 
add_subdirectory(Libraries/YDLidar)
target_link_libraries(ProjectAI ydlidar_sdk)

# spdlog 
add_subdirectory(Libraries/spdlog)
target_include_directories(ProjectAI PRIVATE Libraries/spdlog/include)

# pigpio
target_link_libraries(ProjectAI pigpio)

# glm 
add_subdirectory(Libraries/glm)
target_include_directories(ProjectAI PRIVATE Libraries/glm)

# poco
#add_subdirectory(Libraries/poco)
#target_link_libraries(ProjectAI Poco)
#target_include_directories(ProjectAI PRIVATE Libraries/poco/Net/include)
#target_include_directories(ProjectAI PRIVATE Libraries/poco/Util/include)
