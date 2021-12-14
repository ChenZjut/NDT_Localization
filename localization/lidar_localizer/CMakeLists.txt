cmake_minimum_required(VERSION 2.8.3)
project(lidar_localizer)

find_package(PCL REQUIRED)

if(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
  SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(CUDA)
find_package(Eigen3 QUIET)

if(USE_CUDA)
  add_definitions(-DCUDA_FOUND)
  list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif()

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  ${PCL_OPENMP_PACKAGES}
  autoware_msgs
  jsk_rviz_plugins
  nav_msgs
  ndt_cpu
  ndt_tku
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  velodyne_pointcloud
)

catkin_package(
  CATKIN_DEPENDS
    ${PCL_OPENMP_PACKAGES}
    autoware_msgs
    jsk_rviz_plugins
    ndt_cpu
    ndt_tku
    std_msgs
    velodyne_pointcloud
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching src/ndt_matching_core.cpp
               src/ndt_matching_node.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
add_dependencies(ndt_matching ${catkin_EXPORTED_TARGETS})

if(USE_CUDA)
  target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
endif()


if(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
  set_target_properties(ndt_matching PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif()
