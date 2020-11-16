cmake_minimum_required(VERSION 2.8.8)

# Project name
project(vicon2gt)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag geometry_msgs sensor_msgs nav_msgs)

# Include libraries
find_package(Eigen3 REQUIRED) # built gtsam with cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time serialization regex timer)
find_package(GTSAM REQUIRED) # built gtsam with cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
set(GTSAM_LIBRARIES gtsam)

# Describe catkin project
catkin_package(
    CATKIN_DEPENDS roscpp rosbag geometry_msgs sensor_msgs nav_msgs
    INCLUDE_DIRS src
)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall")


# Include our header files
include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MLK_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${MKL_LIBRARIES}
    ${GTSAM_LIBRARIES}
    ${catkin_LIBRARIES}
)


##################################################
# Make the library
##################################################
add_library(vicon2gt_lib SHARED
    src/gtsam/JPLNavState.cpp
    src/gtsam/JPLQuaternion.cpp
    src/gtsam/RotationXY.cpp
    src/gtsam/ImuFactorCPIv1.cpp
    src/gtsam/MeasBased_ViconPoseTimeoffsetFactor.cpp
    src/meas/Interpolator.cpp
    src/meas/Propagator.cpp
    src/sim/BsplineSE3.cpp
    src/sim/Simulator.cpp
    src/solver/ViconGraphSolver.cpp
)
target_link_libraries(vicon2gt_lib ${thirdparty_libraries})
target_include_directories(vicon2gt_lib PUBLIC src)


##################################################
# Make binary files!
##################################################
add_executable(estimate_vicon2gt src/estimate_vicon2gt.cpp)
target_link_libraries(estimate_vicon2gt vicon2gt_lib ${thirdparty_libraries})

add_executable(run_simulation src/run_simulation.cpp)
target_link_libraries(run_simulation vicon2gt_lib ${thirdparty_libraries})





