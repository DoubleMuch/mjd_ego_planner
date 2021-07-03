#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <fstream>
#include "bits/stdc++.h"
#include <vector>
#include "mosek.h" /* Include the MOSEK definition file. */
using namespace std;
const int jerk_n_=7;
const double g=9.80665;
const double yaw=M_PI/2;
struct waypoint_info
{
    double time;
    Eigen::Vector3d w_pos;
};
struct traj_info{
    waypoint_info start_point;
    waypoint_info end_point;
    double x_ctr_point[jerk_n_];
    double y_ctr_point[jerk_n_];
    double z_ctr_point[jerk_n_];
};
class bezier_traj{
    public:
    void waypoints_bezier_traj(vector< waypoint_info > waypoints,vector <traj_info>  &traj,int &flag);
};