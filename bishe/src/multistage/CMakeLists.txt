project(multistage)
cmake_minimum_required(VERSION 3.0.2)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
# set(CMAKE_BUILD_TYPE Release)
# set(EXTERNAL_INCLUDE_DIRS
#   ${EIGEN3_INCLUDE_DIR}
#   )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # tf
  # pcl_conversions
  # pcl_ros
  nav_msgs
  geometry_msgs
  # actionlib
  roborts_common
  roborts_msgs
  interactive_markers
  roborts_costmap
  std_msgs
  )

# find_package(G2O REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
# find_package(SUITESPARSE REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# find_package(PCL 1.7 REQUIRED)
# find_package(ProtoBuf REQUIRED)
find_package(Boost REQUIRED)

catkin_package()



include_directories(
  # /usr/local/osqp/include 
  # /usr/local/osqp-eigen/include 
${catkin_INCLUDE_DIRS}
)
# link_directories(
#   # /usr/local/osqp/lib
# /usr/local/osqp-eigen/lib)



add_executable(multistage_node
  multistage_node.cpp
  # multistage_node.h
)
target_link_libraries(multistage_node
  # PRIVATE
  # planning::global_planner::a_star_planner
  # roborts_costmap
  osqp::osqp
  OsqpEigen::OsqpEigen
  # OsqpEigen
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GLOG_LIBRARY}
)
add_dependencies(multistage_node
  roborts_msgs_generate_messages)




