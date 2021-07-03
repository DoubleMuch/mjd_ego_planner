#include <iostream>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "bits/stdc++.h"
int main(int argc ,char **argv)
{
    ros::init (argc, argv, "minimum_jerk");
    ros::NodeHandle ph;
    ros::Rate r(100);
    

    int usage_ok = 1, quiet = 0;
    
    Eigen::Vector3d start_point={0,0,0};
    Eigen::Vector3d end_point={15,15,15};
    double end_time=3;
    
    
    int jerk_n=6;  //minimum jerk 系数
    int domin=3;   //x,y,z维度3

    int nx=18;
    double c[] = {0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0} ;   // 目标函数中的线性项，长度为nx的向量
    double xupp[] = {0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0};  
    char ixupp[] = {0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0};

    double xlow[] = {0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0};  
    char ixlow[] = {0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0};  


    const int nnzQ = 6*3;       // 数目与矩阵的下三角矩阵的个数相对应
    int irowQ[] = {3,   
                    4,  4,  
                    5,  5,  5,
                    9,   
                    10, 10, 
                    11, 11, 11,
                    15, 
                    16, 16,
                    17, 17, 17};  // 对称矩阵Q，仅在irowQ,jcolQ和dQ中指定
    int jcolQ[] = { 3,  
                    3,  4,  
                    3,  4,  5,
                    9,  
                    9,  10, 
                    9,  10, 11,
                    15, 
                    15, 16, 
                    15, 16, 17 };  // 矩阵的下三角元素
    double dQ[] = {2*36*end_time,
                   2*72*pow(end_time,2),  2*192*pow(end_time,3),
                   2*60*pow(end_time,3), 2*360*pow(end_time,4),    2*720*pow(end_time,5),
                   2*36*end_time,
                   2*72*pow(end_time,2),  2*192*pow(end_time,3),
                   2*60*pow(end_time,3), 2*360*pow(end_time,4),    2*720*pow(end_time,5),
                   2*36*end_time,
                   2*72*pow(end_time,2),  2*192*pow(end_time,3),
                   2*60*pow(end_time,3), 2*360*pow(end_time,4),    2*720*pow(end_time,5)};


    int my=18; //线性等式约束
    double b[]={0,1,0,end_point[0],3,0, 
                0,2,0,end_point[1],4,0,
                0,0,0,end_point[2],0,0};
    int nnzA = 18*3;             // AX=b矩阵A中non-zeros总数
    
    int irowA[]={0,
                        1,
                            2,
                     3, 3,  3,  3,  3,  3,
                        4,  4,  4,  4,  4,
                            5,  5,  5,  5,
                    6,
                        7,
                            8,
                     9, 9,  9,  9,  9,  9,
                        10,  10,  10,  10,  10,
                            11,  11,  11,  11,
                    12,
                        13,
                            14,
                    15, 15,  15,  15,  15,  15,
                        16,  16,  16,  16,  16,
                            17,  17,  17,  17};            // 等式约束中的A
                            
    int jcolA[]={0,
                        1,
                            2,
                    0,  1,  2,  3,  4,  5,
                        1,  2,  3,  4,  5,
                            2,  3,  4,  5,
                    6,
                        7,
                            8,
                    6,  7,  8,  9,  10,  11,
                        7,  8,  9,  10,  11,
                            8,  9,  10,  11,
                    12,
                        13,
                            14,
                    12, 13, 14, 15, 16, 17,
                        13, 14, 15, 16, 17,
                            14, 15, 16, 17}; 
                            
    double dA[]={1,
                        1,
                            1,
                    1,end_time,pow(end_time,2),pow(end_time,3),pow(end_time,4),pow(end_time,5),
                        1,  2*end_time, 3*pow(end_time,2),4*pow(end_time,3),5*pow(end_time,4),
                            2,  6*end_time, 12*pow(end_time,2), 20*pow(end_time,3),
                    1,
                        1,
                            1,
                    1,end_time,pow(end_time,2),pow(end_time,3),pow(end_time,4),pow(end_time,5),
                        1,  2*end_time, 3*pow(end_time,2),4*pow(end_time,3),5*pow(end_time,4),
                            2,  6*end_time, 12*pow(end_time,2), 20*pow(end_time,3),
                    1,
                        1,
                            1,
                    1,end_time,pow(end_time,2),pow(end_time,3),pow(end_time,4),pow(end_time,5),
                        1,  2*end_time, 3*pow(end_time,2),4*pow(end_time,3),5*pow(end_time,4),
                            2,  6*end_time, 12*pow(end_time,2), 20*pow(end_time,3)};
 
    const int mz = 0;         // 不等式约束的个数
    double *clow = 0;
    char *iclow = 0;

    double *cupp = 0;
    char *icupp =0;

    const int nnzC = 0;       // 多项式不等式约束的个数
    int *irowC = 0;
    int *jcolC = 0;
    double *dC = 0;
    
    QpGenSparseMa27 *qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
    cout<<1<<endl;
    QpGenData *prob = (QpGenData *)qp->copyDataFromSparseTriple(
        c, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, irowA, nnzA, jcolA,
        dA, b, irowC, nnzC, jcolC, dC, clow, iclow, cupp, icupp);
    
    QpGenVars *vars = (QpGenVars *)qp->makeVariables(prob);
    QpGenResiduals *resid = (QpGenResiduals *)qp->makeResiduals(prob);

    GondzioSolver *s = new GondzioSolver(qp, prob);

    cout<<1<<endl;
    if (!quiet)
    {
        cout<<2<<endl;
        s->monitorSelf();
        cout<<3<<endl;
    }
        
    int ierr = s->solve(prob, vars, resid);
    cout<<4<<endl;
    double result[18];
    if (ierr == 0) {
        cout.precision(4);
        cout << "Solution: \n";
        //cout<<vars->value<<endl;
        vars->x->copyIntoArray(result);
        vars->x->writefToStream(cout, "x[%{index}] = %{value}");
    } else {
        cout << "Could not solve the problem.\n";
    }


    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("ooqp_path", 1);
    nav_msgs::Path path;
    path.header.frame_id="/world";
    
    for(int i=0;i<18;i++)
    {
        cout<<i<<" "<<result[i]<<endl;

    }
    int ff;
    cin>>ff;
    for(double i=0;i<end_time;i=i+0.05)
    {
        geometry_msgs::PoseStamped this_pose_stamped;
        double x=result[0]+result[1]*i+result[2]*pow(i,2)+result[3]*pow(i,3)+result[4]*pow(i,4)+result[5]*pow(i,5);
        double y=result[6]+result[7]*i+result[8]*pow(i,2)+result[9]*pow(i,3)+result[10]*pow(i,4)+result[11]*pow(i,5);
        double z=result[12]+result[13]*i+result[15]*pow(i,2)+result[15]*pow(i,3)+result[16]*pow(i,4)+result[17]*pow(i,5);
        this_pose_stamped.header.frame_id="/world";
        this_pose_stamped.pose.position.x = x;
        this_pose_stamped.pose.position.y = y;
        this_pose_stamped.pose.position.z = z;
        path.poses.push_back(this_pose_stamped);
        path_pub.publish(path);
        
    }
    
    
    

}