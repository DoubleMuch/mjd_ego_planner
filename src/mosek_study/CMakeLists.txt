cmake_minimum_required(VERSION 3.0.2)
project(mosek_study)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cmake_modules # 添加 1
)
include_directories(
  bezier
  ${catkin_INCLUDE_DIRS}
  
)
link_directories( ${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1 )

include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)

add_executable(mosek_test 
    src/mosek_test.cpp
)
target_link_libraries(mosek_test ${catkin_LIBRARIES}  mosek64)
add_dependencies(mosek_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(b_spline 
    src/b_spline.cpp
)
add_dependencies(b_spline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bezier 
    src/bezier.cpp
)
add_dependencies(bezier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(bezier_minimumjerk_node 
    src/bezier_minimumjerk_node.cpp bezier/bezier_traj.cpp
)
target_link_libraries(bezier_minimumjerk_node ${catkin_LIBRARIES} mosek64)
add_dependencies(bezier_minimumjerk_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
