# configs
set(CXX_WARNINGS true)
set(SOURCE_DIR src)

# Requirements
cmake_minimum_required(VERSION 3.7.2)

# Project settings
project(SPMatch)	# project name

set(CMAKE_CXX_STANDARD 11) # cxx version
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # commands for vim youcompleteme

if(CXX_WARNINGS)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0")
else(CXX_WARNINGS)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif(CXX_WARNINGS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # Linux specific

# targets and files
file(GLOB_RECURSE CPP_SOURCES "${SOURCE_DIR}/*.cpp")

include_directories(include)

add_executable(spmatch ${CPP_SOURCES})

target_link_libraries(spmatch boost_program_options)
target_link_libraries(spmatch X11) # Linux/iOS specific
