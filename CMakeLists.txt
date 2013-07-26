cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_circle)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
# I don't wanna see all the warnings about PCL-SHOT
# enable c0x support (shared_ptr and etc.)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wno-deprecated-declarations")

# I built PCL from source. You may want to comment out this set(PCL_DIR) if
# you got pcl from apt-get
#set(PCL_DIR "/home/ming/Libs/PCL-1.6.0-Source/build")
find_package(PCL 1.5 REQUIRED PATHS /usr NO_DEFAULT_PATH)

#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDES})

include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executables
#add_executable (test_bgs src/test_bgs.cpp)
#target_link_libraries (test_bgs ${PCL_LIBRARIES})

#add_executable (seg_sphere src/seg_sphere.cpp)
#target_link_libraries (seg_sphere ${PCL_LIBRARIES})

# Quick visualization of a given pcd file
#add_executable (show_pcd src/show_pcd.cpp)
#target_link_libraries (show_pcd ${PCL_LIBRARIES})

# Capture data from OpenNI device (Kinect or Xtion Pro)
add_executable (openni_ss src/openni_ss.cpp)
target_link_libraries (openni_ss ${PCL_LIBRARIES})

#add_executable (test_ballfilter src/test_ballfilter.cpp)
#target_link_libraries (test_ballfilter ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (calib3D src/calib3D.cpp)
#target_link_libraries (calib3D ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (downsample src/downsample.cpp)
#target_link_libraries (downsample ${PCL_LIBRARIES})

#add_executable (test_be src/test_be.cpp)
#target_link_libraries(test_be ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (test_calib3D src/test_calib3D.cpp)
#target_link_libraries(test_calib3D ${CERES_LIBRARIES})

#add_executable (test_pipeline src/test_pipeline.cpp)
#target_link_libraries(test_pipeline ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (show_calib_result src/show_calib_result.cpp)
#target_link_libraries (show_calib_result ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (test_f src/test_f.cpp)
#target_link_libraries (test_f ${PCL_LIBRARIES})

#add_executable (test_2ndrun src/test_2ndrun.cpp)
#target_link_libraries(test_2ndrun ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_subdirectory(test)