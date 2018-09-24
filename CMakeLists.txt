
# Requirements
cmake_minimum_required(VERSION 3.7.2)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Project settings
project(SPMatch)	# project name

set(CMAKE_CXX_STANDARD 11) # cxx version
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # commands for vim youcompleteme

# Build types
# You can set a build type calling cmake with
# cmake -DCMAKE_BUILD_TYPE=Debug ../..			# or
# cmake -DCMAKE_BUILD_TYPE=Release ../..
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -O0") # -g implicit

# Linux specific
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# targets and files
file(GLOB CPP_SOURCES "src/*.cpp")

include_directories(include)

add_executable(spmatch ${CPP_SOURCES})
target_link_libraries(spmatch boost_program_options)
target_link_libraries(spmatch Eigen3::Eigen)
target_link_libraries(spmatch X11) # Linux/iOS specific
