cmake_minimum_required(VERSION 3.5)
project(contrast_maximization)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g ${CMAKE_CXX_FLAGS}")

add_definitions("-DCATKIN_ENABLE_TESTING=0")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

## find all packages needed
include(${PROJECT_SOURCE_DIR}/cmake/packages.cmake)

# include directories needed
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

#add_executable(cm_translation_2d src/cm_translation_2d.cpp)
#target_link_libraries(cm_translation_2d ${third_party_libs})

add_executable(cm_rotation_3d src/cm_rotation_3d.cpp)
target_link_libraries(cm_rotation_3d ${third_party_libs})

#add_executable(pcd_to_bag src/pcd_to_bag.cpp)
#target_link_libraries(pcd_to_bag ${third_party_libs})
