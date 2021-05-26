/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

GlobalOptimization globalEstimator;
GlobalOptimization globalEstimator2;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
nav_msgs::Path *gps_path_global;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;
ofstream rtk_file;
ofstream gps_file;
ofstream fusion_file;


std::vector<Eigen::Vector3d> rtk_traj_xyz;
void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg,ros::Publisher *path_pub,nav_msgs::Path *path)
{
    //printf("gps_callback! \n");
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();


    double gps_t = GPS_msg->header.stamp.toSec();
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = 1;
    //int numSats = GPS_msg->status.service;
    double pos_accuracy = GPS_msg->position_covariance[0];
    if(pos_accuracy <= 0)
        pos_accuracy = 1;
    //printf("receive covariance %lf \n", pos_accuracy);
    //if(GPS_msg->status.status > 8)
    double xyz[3];
    globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
    
    //std::cout<<"gps "<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<std::endl;
    geometry_msgs::PoseStamped this_pose_stamped;
    ros::Time current_time = ros::Time::now();
    this_pose_stamped.header.frame_id="/world";
    this_pose_stamped.pose.position.x = xyz[0];
    this_pose_stamped.pose.position.y = xyz[1];
    this_pose_stamped.pose.position.z = xyz[2];
//    this_pose_stamped.pose.position.x = a++;
//    this_pose_stamped.pose.position.y = a++;
//    this_pose_stamped.pose.position.z = a++;
    this_pose_stamped.header.stamp=GPS_msg->header.stamp;
    path->poses.push_back(this_pose_stamped);
    path_pub->publish(*path);

    gps_file.precision(9);
    gps_file << GPS_msg->header.stamp.toSec()<< " ";
    gps_file << xyz[0] <<" "<< xyz[1] <<" "<< xyz[2] <<" "<< "1"<<" "<< "1"<<" "<< "1"<<" "<< "1"<<endl;
    // m_current_xyz.lock();
    // current_gps_xyz[0]=xyz[0];current_gps_xyz[1]=xyz[1];current_gps_xyz[2]=xyz[2];
    // m_current_xyz.unlock();
}
void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg,ros::Publisher *path_pub,nav_msgs::Path *path)
{
   
    double gps_t = msg->header.stamp.toSec();
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    double altitude = 1;
    //int numSats = GPS_msg->status.service;
    double pos_accuracy = msg->position_covariance[0];
    if(pos_accuracy <= 0)
        pos_accuracy = 1;
    //printf("receive covariance %lf \n", pos_accuracy);
    //if(GPS_msg->status.status > 8)
    double xyz[3];
    globalEstimator2.GPS2XYZ(latitude, longitude, altitude, xyz);
    xyz[2]=0;
    std::cout<<"rtk "<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<std::endl;
    geometry_msgs::PoseStamped this_pose_stamped;
    ros::Time current_time = ros::Time::now();
    this_pose_stamped.header.frame_id="/world";
    this_pose_stamped.pose.position.x = xyz[0];
    this_pose_stamped.pose.position.y = xyz[1];
    this_pose_stamped.pose.position.z = xyz[2];
//    this_pose_stamped.pose.position.x = a++;
//    this_pose_stamped.pose.position.y = a++;
//    this_pose_stamped.pose.position.z = a++
    this_pose_stamped.header.stamp=msg->header.stamp;
    path->poses.push_back(this_pose_stamped);
    path_pub->publish(*path);


    rtk_file.precision(9);
    rtk_file << msg->header.stamp.toSec() << " ";
    rtk_file << xyz[0] <<" "<< xyz[1] <<" "<< xyz[2] <<" "<< "1"<<" "<< "1"<<" "<< "1"<<" "<< "1"<<endl;
    //calcuate gps_error
    // m_current_xyz.lock();
    // double temp_error_2d=sqrt(pow(xyz[0]-current_gps_xyz[0],2)+pow(xyz[1]-current_gps_xyz[1],2));
    // m_current_xyz.unlock();
    // gps_rtk_mean_error=(point_num*gps_rtk_mean_error+temp_error_2d)/(point_num+1);
    // point_num++;
    // cout<<"gps_rtk_mean_error "<<gps_rtk_mean_error<<endl;

    //calcuate vslam_error
   
    int global_size=global_path->poses.size();
    int rtk_size=path->poses.size();
    int gps_size=gps_path_global->poses.size();
    cout<<"global_size  "<<global_size<<endl;
    cout<<"rtk_size  "<<rtk_size<<endl;
    cout<<"gps_size  "<<gps_size<<endl;
    double count=0;
    double fusion_rtk_mean_error=0;
    double gps_rtk_mean_error=0;
    double point_num=0;
    if(global_size>0 && rtk_size>0 && gps_size>0 )
    {    
        int global_index=0;
        int gps_index=0;
        for(int i=0;i<rtk_size;i++)
        {
            double rtk_t = path->poses[i].header.stamp.toSec();
            int next_index;
            double t_slot=fabs(global_path->poses[global_index].header.stamp.toSec()-rtk_t);
            while(global_index<global_size)
            {
                next_index=global_index+1;
                t_slot=fabs(global_path->poses[global_index].header.stamp.toSec()-rtk_t);
                
                if(next_index >= global_size )
                {
                    i=rtk_size;
                    break;
                }
                double next_global_t=global_path->poses[next_index].header.stamp.toSec();
                double t_slot_next=fabs(next_global_t -rtk_t);
                if(t_slot <= t_slot_next )
                {
                    double global_xyz[3];
                    double rtk_xyz[3];
                    global_xyz[0]=global_path->poses[global_index].pose.position.x;
                    global_xyz[1]=global_path->poses[global_index].pose.position.y;
                    global_xyz[2]=global_path->poses[global_index].pose.position.z;
                    rtk_xyz[0]=path->poses[i].pose.position.x;
                    rtk_xyz[1]=path->poses[i].pose.position.y;
                    rtk_xyz[2]=path->poses[i].pose.position.z;
                    fusion_rtk_mean_error+=sqrt(pow(rtk_xyz[0]-global_xyz[0],2)+pow(rtk_xyz[1]-global_xyz[1],2));
                    count++;    
                    
                    cout<<"t_slot "<<t_slot<<endl;
                    double gps_t_slot=fabs(gps_path_global->poses[gps_index].header.stamp.toSec()-global_path->poses[global_index].header.stamp.toSec());
                    while((gps_index+1)<gps_size)
                    {
                        gps_t_slot=fabs(gps_path_global->poses[gps_index].header.stamp.toSec()-global_path->poses[global_index].header.stamp.toSec());
                        double gps_t_slot_next=fabs(gps_path_global->poses[gps_index+1].header.stamp.toSec()-global_path->poses[global_index].header.stamp.toSec());
                        if(gps_t_slot<=gps_t_slot_next)
                        {
                            double gps_xyz[3];
                            cout<<"gps_t_slot "<<gps_t_slot<<endl;
                            gps_xyz[0]=gps_path_global->poses[gps_index].pose.position.x;
                            gps_xyz[1]=gps_path_global->poses[gps_index].pose.position.y;
                            gps_xyz[2]=gps_path_global->poses[gps_index].pose.position.z;
                            gps_rtk_mean_error+=sqrt(pow(rtk_xyz[0]-gps_xyz[0],2)+pow(rtk_xyz[1]-gps_xyz[1],2));
                            point_num++;
                            gps_index++;
                            break;
                        }
                        
                        gps_index++;
                    }
                    
                    


                    global_index++;
                    break;       
                }
                else 
                {
                    global_index++;
                    
                }
                // if(global_t < rtk_t)
                // {
                //     if(global_path->poses[next_index].header.stamp.toSec() < rtk_t)
                //         global_index++;
                //     else
                //     {
                //         double t_slot1=fabs(global_t - rtk_t);
                //         double t_slot2=fabs(global_path->poses[next_index].header.stamp.toSec() - rtk_t);
                //         if(t_slot1 > t_slot2)
                //             global_index++;
                //         double global_xyz[3];
                //         double rtk_xyz[3];
                //         global_xyz[0]=global_path->poses[global_index].pose.position.x;
                //         global_xyz[1]=global_path->poses[global_index].pose.position.y;
                //         global_xyz[2]=global_path->poses[global_index].pose.position.z;
                //         rtk_xyz[0]=path->poses[i].pose.position.x;
                //         rtk_xyz[1]=path->poses[i].pose.position.y;
                //         rtk_xyz[2]=path->poses[i].pose.position.z;
                //         fusion_rtk_mean_error+=sqrt(pow(rtk_xyz[0]-global_xyz[0],2)+pow(rtk_xyz[1]-global_xyz[1],2));
                //         count++;
                //         break;
                //     }
                // }
                // else
                // {
                //     break;
                // }

                
            }
            if(global_index>=global_size)
                break;

        }
        //cout<<"position.x "<<global_path->poses[global_size-1].pose.position.x<<endl;
    }
    if(count > 0)
    {
        cout<<"fusion_rtk_mean_error "<<fusion_rtk_mean_error/count<<endl;
        cout<<"count "<<count<<endl;
        cout<<"gps_rtk_mean_error "<<gps_rtk_mean_error/point_num<<endl;
        cout<<"gps_count "<<point_num<<endl;
       

    }
   

}
void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, 0);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);


    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
                globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 0.01)
            gpsQueue.pop();
        else if(gps_t > t + 0.01)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);

    
    // write result to file
    std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();


    fusion_file.open("/home/dji/data/tum_fusion.txt",  ios::out );
    fusion_file.setf(ios::fixed, ios::floatfield);
    fusion_file.precision(9);
    for(int i=0;i<global_path->poses.size();i++)
    {
        
        fusion_file << global_path->poses[i].header.stamp.toSec() << " ";
        fusion_file << global_path->poses[i].pose.position.x <<" "<< global_path->poses[i].pose.position.y <<" "<< global_path->poses[i].pose.position.z <<" "<< 1<<" "<< 1 <<" "<< 1 <<" "<< 1<<endl;

        //fusion_file << global_path->poses[i].pose.position.x <<" "<< global_path->poses[i].pose.position.y <<" "<< global_path->poses[i].pose.position.z <<" "<< global_path->poses[i].pose.orientation.x<<" "<< global_path->poses[i].pose.orientation.y <<" "<< global_path->poses[i].pose.orientation.z <<" "<< global_path->poses[i].pose.orientation.w<<endl;
    }
    fusion_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");
    rtk_file.open("/home/dji/data/tum_rtk.txt",  ios::out  );
    gps_file.open("/home/dji/data/tum_gps.txt",  ios::out );
    
    rtk_file.setf(ios::fixed, ios::floatfield);
    gps_file.setf(ios::fixed, ios::floatfield);
    
    global_path = &globalEstimator.global_path;
    
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);


    ros::Publisher gps_path_pub = n.advertise<nav_msgs::Path>("gps_path", 1);
    nav_msgs::Path gps_path;
    gps_path_global=&gps_path;
    gps_path.header.frame_id="/world";
    ros::Subscriber sub_gps=n.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/gps_position", 1,  boost::bind(&GPS_callback,_1,&gps_path_pub,&gps_path));
    ros::Publisher rtk_path_pub = n.advertise<nav_msgs::Path>("rtk_path", 1);
    nav_msgs::Path rtk_path;
    rtk_path.header.frame_id="/world";
    ros::Subscriber sub_rtk=n.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/rtk_position", 1,  boost::bind(&rtkCallback,_1,&rtk_path_pub,&rtk_path));
    ros::spin();
    return 0;
}
