cmake_minimum_required(VERSION 3.0.2)
project(ooqp_study)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
# include
  ${catkin_INCLUDE_DIRS}
  /home/dji/OOQP/include
  src/trajectory_generator
)

link_directories(/home/dji/OOQP/lib)
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)
add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ooqpgensparse
  ooqpsparse
  ooqpgondzio
  ooqpbase blas ma27 gfortran)
add_dependencies(test1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(minimum_jerk src/minimum_jerk.cpp src/trajectory_generator/trajectory_generator.cpp)

target_link_libraries(minimum_jerk ooqpgensparse
  ooqpsparse
  ooqpgondzio
  ooqpbase blas ma27 gfortran
   ${catkin_LIBRARIES})
add_dependencies(minimum_jerk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(main_node src/main.cpp src/trajectory_generator/trajectory_generator.cpp)

target_link_libraries(main_node ooqpgensparse
  ooqpsparse
  ooqpgondzio
  ooqpbase blas ma27 gfortran
   ${catkin_LIBRARIES})
add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





