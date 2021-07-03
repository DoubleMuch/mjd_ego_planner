#pragma once
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "bits/stdc++.h"
#include <ros/ros.h>
#include <ros/console.h>
const int jerk_n_=6;
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
    double x_coeff[jerk_n_];
    double y_coeff[jerk_n_];
    double z_coeff[jerk_n_];
};
class trajectory_generator{
    public:
    void waypoints_make_traj(vector< waypoint_info > waypoints,vector <traj_info>  &traj,int &flag);
};