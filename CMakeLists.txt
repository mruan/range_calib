cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_circle)

set(PCL_DIR "/home/ming/Libs/PCL-1.6.0-Source/build")
find_package(PCL 1.2 REQUIRED PATHS /usr NO_DEFAULT_PATH)

include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executables
add_executable (filterBG src/filterBG.cpp)
target_link_libraries (filterBG ${PCL_LIBRARIES})

add_executable (seg_sphere src/seg_sphere.cpp)
target_link_libraries (seg_sphere ${PCL_LIBRARIES})

add_executable (downsample src/downsample.cpp)
target_link_libraries (downsample ${PCL_LIBRARIES})

add_executable (show_pcd src/show_pcd.cpp)
target_link_libraries (show_pcd ${PCL_LIBRARIES})

add_executable (openni_ss src/openni_ss.cpp)
target_link_libraries (openni_ss ${PCL_LIBRARIES})

add_executable (test_ballfilter src/test_ballfilter.cpp src/ball_filter.cpp)
target_link_libraries (test_ballfilter ${PCL_LIBRARIES})
