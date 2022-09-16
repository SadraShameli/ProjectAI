# Adding Main.cpp as excutable file

file(GLOB SRC_FILES *.cpp)
add_executable(ProjectAI Main.cpp ${SRC_FILES})
