cmake_minimum_required(VERSION 3.0.2)
project(mjd_fusion)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cmake_modules # 添加 1
)



find_package(Ceres REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	./ThirdParty/GeographicLib/include/
)
catkin_package(
        INCLUDE_DIRS include
        #        LIBRARIES GenerativeModel
        CATKIN_DEPENDS
)
add_executable(gps_tra_node
        src/gps_tra.cpp
		src/globalOpt.cpp
        )
add_dependencies(gps_tra_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_tra_node  ${catkin_LIBRARIES} ${CERES_LIBRARIES} libGeographicccmjd)
