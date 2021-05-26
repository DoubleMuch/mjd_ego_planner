#include "bits/stdc++.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "globalOpt.h"

double a=1;
GlobalOptimization globalEstimator;
GlobalOptimization globalEstimator2;
sensor_msgs::ImuConstPtr latest_imu_msg;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;
std::mutex m_current_xyz;
double gps_rtk_mean_error=0;
double point_num=0;
double current_gps_xyz[3];
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg,ros::Publisher *path_pub,nav_msgs::Path *path)
{
    m_buf.lock();
    gpsQueue.push(msg);
    m_buf.unlock();


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
    globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
    if(a<2)
    {
        std::cout<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<std::endl;
        std::cout<<setprecision(10)<<msg->latitude<<" "<<longitude<<" "<<altitude<<std::endl;
    }
    a++;
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
    this_pose_stamped.header.stamp=current_time;
    path->poses.push_back(this_pose_stamped);
    path_pub->publish(*path);

    m_current_xyz.lock();
    current_gps_xyz[0]=xyz[0];current_gps_xyz[1]=xyz[1];current_gps_xyz[2]=xyz[2];
    m_current_xyz.unlock();
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
    globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
    if(a<2)
    {
        std::cout<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<std::endl;
        std::cout<<setprecision(10)<<msg->latitude<<" "<<longitude<<" "<<altitude<<std::endl;
    }
    a++;
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
    this_pose_stamped.header.stamp=current_time;
    path->poses.push_back(this_pose_stamped);
    path_pub->publish(*path);

    //calcuate gps_error
    m_current_xyz.lock();
    double temp_error_2d=sqrt(pow(xyz[0]-current_gps_xyz[0],2)+pow(xyz[1]-current_gps_xyz[1],2));
    m_current_xyz.unlock();
    gps_rtk_mean_error=(point_num*gps_rtk_mean_error+temp_error_2d)/(point_num+1);
    point_num++;
    cout<<"gps_rtk_mean_error "<<gps_rtk_mean_error<<endl;



}
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{

    latest_imu_msg=imu_msg;
    //cout<<latest_imu_msg->orientation.x<<endl;
    //latest_imu_msg.push(imu_msg);
}
void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    cout<<"latest_imu_msg"<<endl;
    cout<<latest_imu_msg->orientation.x<<endl;
    cout<<"vio_message"<<endl;
    cout<<pose_msg->pose.pose.orientation.x<<endl;
    Eigen::Quaterniond imuQ;
    imuQ.w()=latest_imu_msg->orientation.w;
    imuQ.x()=latest_imu_msg->orientation.x;
    imuQ.y()=latest_imu_msg->orientation.y;
    imuQ.z()=latest_imu_msg->orientation.z;

    double t = pose_msg->header.stamp.toSec();
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vioQ;
    vioQ.w()=pose_msg->pose.pose.orientation.w;
    vioQ.x()=pose_msg->pose.pose.orientation.x;
    vioQ.y()=pose_msg->pose.pose.orientation.y;
    vioQ.z()=pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vioQ);
{
    Eigen::Matrix3d imu_rotation_matrix;
    imu_rotation_matrix=imuQ.toRotationMatrix();

    Eigen::Matrix3d vio_rotation_matrix;
    vio_rotation_matrix=vioQ.toRotationMatrix();

    Eigen::Matrix3d vio_T_imu;
    vio_T_imu=imu_rotation_matrix*vio_rotation_matrix.inverse();
    
    cout<<"vio_T_imu"<<endl;
    cout<<vio_T_imu<<endl;

    cout<<"vio_rotation_matrix"<<endl;
    cout<<vio_rotation_matrix<<endl;

    cout<<"imu_rotation_matrix"<<endl;
    cout<<imu_rotation_matrix<<endl;
}
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
    

}
int main(int argc ,char **argv)
{
    
    ros::init (argc, argv, "gps_tra_node");
    ros::NodeHandle ph;
    ros::Rate r(100);
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("gps_path", 1);
    nav_msgs::Path path;
    ros::Publisher rtk_path_pub = ph.advertise<nav_msgs::Path>("rtk_path", 1);
    nav_msgs::Path rtk_path;
    path.header.frame_id="/world";
    rtk_path.header.frame_id="/world";
    std::cout<<1<<std::endl;
    ros::Subscriber sub_pos=ph.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/gps_position", 1,  boost::bind(&gpsCallback,_1,&path_pub,&path));
    ros::Subscriber sub_pos2=ph.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/rtk_position", 1,  boost::bind(&rtkCallback,_1,&rtk_path_pub,&rtk_path));
    //ros::Subscriber sub_imu=ph.subscribe<sensor_msgs::Imu>("/djiros/imu",1, imuCallback);
    //ros::Subscriber sub_vio = ph.subscribe("/vins_estimator/odometry", 100, vio_callback);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}