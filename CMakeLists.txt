# configs
set(DEBUGGING true)
set(CPP_SOURCES src/spmatch.cpp src/image.cpp src/geometry.cpp)
set(CPP_SOURCES_DEB src/debug.cpp src/image.cpp src/geometry.cpp)

# Requirements
cmake_minimum_required(VERSION 3.7.2)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Project settings
project(SPMatch)	# project name

set(CMAKE_CXX_STANDARD 11) # cxx version
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # commands for vim youcompleteme

if(DEBUGGING)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0")
else(DEBUGGING)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif(DEBUGGING)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # Linux specific

# targets and files

include_directories(include)

# main source
add_executable(spmatch ${CPP_SOURCES})
target_link_libraries(spmatch boost_program_options)
target_link_libraries(spmatch X11) # Linux/iOS specific
target_link_libraries(spmatch Eigen3::Eigen)

# testing
add_executable(spmatch_deb ${CPP_SOURCES_DEB})
target_link_libraries(spmatch_deb boost_program_options)
target_link_libraries(spmatch_deb X11) # Linux/iOS specific
target_link_libraries(spmatch_deb Eigen3::Eigen)
