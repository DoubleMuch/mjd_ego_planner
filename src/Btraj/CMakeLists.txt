cmake_minimum_required(VERSION 2.8.3)
project(bezier_planer)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    quadrotor_msgs
    tf
    rospy
    cv_bridge
    message_filters
    geometry_msgs
    plan_env
    path_searching
    traj_utils
    message_generation
)

#set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bezier_planer
  CATKIN_DEPENDS plan_env path_searching traj_utils message_runtime
)

link_directories( ${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1 )

include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)


include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(b_traj_node 
    src/b_traj_node.cpp
    src/bezier_base.cpp
    src/trajectory_generator.cpp
    third_party/fast_methods/console/console.cpp
    third_party/fast_methods/fm/fmdata/fmcell.cpp
    third_party/fast_methods/ndgridmap/cell.cpp
)
target_link_libraries(b_traj_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES} mosek64)


add_executable ( b_traj_server src/traj_server.cpp src/bezier_base.cpp)
target_link_libraries( b_traj_server
                        ${catkin_LIBRARIES}
)

add_executable ( random_forest_sensing src/random_forest_sensing.cpp )
target_link_libraries( random_forest_sensing
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

add_executable ( odom_generator src/odom_generator.cpp )
target_link_libraries( odom_generator
                        ${catkin_LIBRARIES}
)