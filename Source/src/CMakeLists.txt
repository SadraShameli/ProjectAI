# Adding Main.cpp and all .cpp files as executable file

file(GLOB SRC_FILES *.cpp)
add_executable(ProjectAI ${SRC_FILES})
