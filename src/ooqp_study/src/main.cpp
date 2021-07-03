#include "trajectory_generator.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <fstream>

//解决转欧拉角不稳定 参考网站 https://zhuanlan.zhihu.com/p/55790406
void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
if (fabs(sinp) >= 1)
pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
else
pitch = asin(sinp);

// yaw (z-axis rotation)
double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}

void RationMatrixtoEulerAngle(const Eigen::Matrix3d& Ration, double& roll, double& pitch, double& yaw)
{
    //yaw 在-pi到pi之间，可以取0， 不能取到端点  roll，pitch在-pi/2到pi/2之间, 不能取到端点
    
    
    yaw=atan2(Ration(1,0),Ration(0,0));
    
    pitch=atan2(-Ration(2,0),sqrt(pow(Ration(2,1),2)+pow(Ration(2,2),2)));
    
    roll=atan2(Ration(2,1),Ration(2,2));
}


int main(int argc ,char **argv)
{
    //参数初始化
    //double a[10000][10000]={1};
    ros::init (argc, argv, "main");
    ros::NodeHandle ph;
    ros::Rate r(10);
    ofstream fs;
    fs.open("/home/dji/catkin_ws/src/ooqp_study/date/data.txt");
    std::vector<double> point_time;
    std::vector<double> point_pos;
    ph.getParam("waypoints",point_pos);
    ph.getParam("time_stamp",point_time);
    int waypoint_num=point_time.size();
    double way_points[waypoint_num][3];
    for(int i=0;i<waypoint_num;i++)
    {
        way_points[i][0]=point_pos[i*3+0];
        way_points[i][1]=point_pos[i*3+1];
        way_points[i][2]=point_pos[i*3+2];
        cout<<way_points[i][0]<<" "<<way_points[i][1]<<" "<<way_points[i][2]<<endl;
    }
    double time_slots[waypoint_num-1];
    vector<waypoint_info> w;
    vector<traj_info> traj;
    vector<vector <double> > coeff;
    for(int i=0;i<waypoint_num;i++)
    {
        Eigen::Vector3d point={way_points[i][0],way_points[i][1],way_points[i][2]};
        waypoint_info p;
        p.w_pos=point;
        p.time=point_time[i];
        w.push_back(p);
        if(i<(waypoint_num-1))
        {
            time_slots[i]=point_time[i+1]-point_time[i];
            vector<double> x(6);
        }
        if(i>=1)
        {
            traj_info t;
            t.start_point=w[i-1];
            t.end_point=w[i];
            traj.push_back(t);
        }
    }

    //计算多项式系数
    int flag=1;
    trajectory_generator Tra;
    Tra.waypoints_make_traj(w,traj,flag);
    if(flag==0)
        return 0;

    //test rviz_visual_tools
    
    // Allow the action server to recieve and send ros messages
   
   
    //rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    // std::string s1="world";
    // std::string s2="rviz_visual_markers";
    //rviz_visual_tools::RvizVisualTools aaa("world","/rviz_visual_markers");
    //visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));
    

    //根据结果画轨迹
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("ooqp_path", 1);
    ros::Publisher pose_pub = ph.advertise<geometry_msgs::PoseStamped>("poses", 1);
    ros::Publisher marker_pub = ph.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
    nav_msgs::Path path;
    path.header.frame_id="/world";
    double  ff=g;
    cin>>ff;

    for(int i=0;i<w.size();i++)
    {
        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker marker;
        // 设置帧 ID和时间戳
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint_marker_"+char(i);
        marker.id = i;
        marker.type = shape;
        // 设置标记行为：ADD（添 加），DELETE（删 除）
        //marker.action = visualization_msgs::Marker::ADD;
        //设置标记位姿。 
        marker.pose.position.x = w[i].w_pos[0];
        marker.pose.position.y = w[i].w_pos[1];
        marker.pose.position.z = w[i].w_pos[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // 设置标记的比例，所有方向上尺度1表示1米
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        //cout<<marker.pose.position.x<<" "<<marker.pose.position.y<<" "<<marker.pose.position.z<<endl;
        //设置标记颜色，确保alpha（不透明度）值不为0
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        r.sleep();
        marker_pub.publish(marker);
    }
    cout<<w.size()<<endl;
    cin>>ff;
    for(int i=0;i<6;i++)
    {
        cout<<traj[0].x_coeff[i]<<" ";
    }
    cout<<endl;
    //int j=-1;
    // while(ros::ok())
    // {
    //     j++;
    //     if(j>=(waypoint_num-1))
    //         break;
    // }
    for(int j=0;j<(waypoint_num-1);j++)
    {
        for(double i=0;i<time_slots[j];i=i+0.05)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            double x=traj[j].x_coeff[0]+traj[j].x_coeff[1]*i+traj[j].x_coeff[2]*pow(i,2)+traj[j].x_coeff[3]*pow(i,3)+traj[j].x_coeff[4]*pow(i,4)+traj[j].x_coeff[5]*pow(i,5);
            double y=traj[j].y_coeff[0]+traj[j].y_coeff[1]*i+traj[j].y_coeff[2]*pow(i,2)+traj[j].y_coeff[3]*pow(i,3)+traj[j].y_coeff[4]*pow(i,4)+traj[j].y_coeff[5]*pow(i,5);
            double z=traj[j].z_coeff[0]+traj[j].z_coeff[1]*i+traj[j].z_coeff[2]*pow(i,2)+traj[j].z_coeff[3]*pow(i,3)+traj[j].z_coeff[4]*pow(i,4)+traj[j].z_coeff[5]*pow(i,5);
            double vx=traj[j].x_coeff[1]+2*traj[j].x_coeff[2]*i+3*traj[j].x_coeff[3]*pow(i,2)+4*traj[j].x_coeff[4]*pow(i,3)+5*traj[j].x_coeff[5]*pow(i,4);
            double vy=traj[j].y_coeff[1]+2*traj[j].y_coeff[2]*i+3*traj[j].y_coeff[3]*pow(i,2)+4*traj[j].y_coeff[4]*pow(i,3)+5*traj[j].y_coeff[5]*pow(i,4);
            double vz=traj[j].z_coeff[1]+2*traj[j].z_coeff[2]*i+3*traj[j].z_coeff[3]*pow(i,2)+4*traj[j].z_coeff[4]*pow(i,3)+5*traj[j].z_coeff[5]*pow(i,4);
            double ax=2*traj[j].x_coeff[2]+6*traj[j].x_coeff[3]*i+12*traj[j].x_coeff[4]*pow(i,2)+20*traj[j].x_coeff[5]*pow(i,3);
            double ay=2*traj[j].y_coeff[2]+6*traj[j].y_coeff[3]*i+12*traj[j].y_coeff[4]*pow(i,2)+20*traj[j].y_coeff[5]*pow(i,3);
            double az=2*traj[j].z_coeff[2]+6*traj[j].z_coeff[3]*i+12*traj[j].z_coeff[4]*pow(i,2)+20*traj[j].z_coeff[5]*pow(i,3);
            double cur_t=point_time[j]+i;
            
            Eigen::Matrix3d R2;     //R z,x,y
            R2= Eigen::Matrix3d::Identity(); 
            Eigen::AngleAxisd rotation_vector3 ( 0 ,Eigen::Vector3d ( 0,0,1 ) );  //yaw
            R2=R2*rotation_vector3.toRotationMatrix();
            Eigen::AngleAxisd rotation_vector ( 0, Eigen::Vector3d ( 1,0,0 ) );  //roll
            R2=R2*rotation_vector.toRotationMatrix();
            Eigen::AngleAxisd rotation_vector2 ( 0, Eigen::Vector3d ( 0,1,0 ) );  //pitch
            R2=R2*rotation_vector2.toRotationMatrix();

            Eigen::Vector3d Z_B={ax,ay,az+g};
            // Eigen::Vector3d Z_B={R2(0,2),R2(1,2),R2(2,2)};
            
            Z_B=Z_B/(Z_B.norm());
            Eigen::Vector3d X_C={cos(yaw),sin(yaw),0};
            
            //Eigen::Vector3d Y_B ={ Z_B[1]*X_C[2]-X_C[1]*Z_B[2],Z_B[2]*X_C[0]-X_C[2]*Z_B[1],   Z_B[0]*X_C[1]-X_C[0]*Z_B[1]               };
            Eigen::Vector3d Y_B=Z_B.cross(X_C)/((Z_B.cross(X_C)).norm());
            Y_B=Y_B/(Y_B.norm());
            //Eigen::Vector3d X_B=Y_B.cross(Z_B);
            Eigen::Vector3d X_B=Y_B.cross(Z_B);
            Eigen::Matrix3d R;
            R.col(0)=X_B;
            R.col(1)=Y_B;
            R.col(2)=Z_B;
            double roll,yaw,pitch;
            cout <<  "********** EulerAngle **********" << endl;
            
            
            
            // Eigen::Vector3d vec1={1,2,3};
            // Eigen::Vector3d vec2={2,3,4};
            // Eigen::Vector3d vec1_R=(R*vec1);
            // Eigen::Vector3d vec2_R=(R*vec2);
            // cout<<"mult "<< vec1.dot(vec2) <<" "<<vec1_R.dot(vec2_R)<<endl;
            // cout<<"R2"<<endl;
            // cout<<R2<<endl;
            // cout<<"R"<<endl;
            // cout<<R<<endl;
            // cout<<"roll "<< asin(Y_B[2])<<endl;
            // cout<<"X_C "<<X_C.transpose()<<endl;
            // cout<<"X_B "<<X_B.transpose()<<endl;
            // cout<<"Y_B "<<Y_B.transpose()<<endl;
            // cout<<"Z_B "<<Z_B.transpose()<<endl;


            //3.0 初始化欧拉角(Y-P-R YAW,PITCH,ROLL)  YAW角为0，朝向世界坐标系x轴正方形，红色轴为机头朝向
            // Eigen::Vector3d ea(0, 0,0);
            // Eigen::Quaterniond quaternion3;
            // Eigen::Matrix3d Ration;
            // Ration =   Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
            //                 Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
            //                 Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
            // quaternion3=Ration;
            
            // cout<<"X_B "<<Ration.col(0).transpose()<<endl;
            // //cout<<"yaw "<<acos(Ration.col(0)[0]/sqrt(pow(Ration.col(0)[0],2)+pow(Ration.col(0)[1],2)))<<endl;
            // cout<<"yaw "<<atan2(Ration(1,0),Ration(0,0))<<endl;

            // cout<<"Y_B "<<Ration.col(1).transpose()<<endl;
            // //cout<<"pitch "<<asin(Ration.col(0)[2])<<endl;
            // cout<<"pitch "<<atan2(-Ration(2,0),sqrt(pow(Ration(2,1),2)+pow(Ration(2,2),2)))<<endl;
            
            // cout<<"Z_B "<<Ration.col(2).transpose()<<endl;
            // //cout<<"roll "<<asin(Ration.col(2)[2])<<endl;
            // cout<<"roll "<<atan2(Ration(2,1),Ration(2,2))<<endl;
            

            //红色为x轴，绿色为y轴，蓝色为z轴。
            Eigen::Quaterniond qua(R);
            this_pose_stamped.header.frame_id="/world";
            this_pose_stamped.pose.position.x = x;
            this_pose_stamped.pose.position.y = y;
            this_pose_stamped.pose.position.z = z;
            this_pose_stamped.pose.orientation.x = qua.x();
            this_pose_stamped.pose.orientation.y = qua.y();
            this_pose_stamped.pose.orientation.z = qua.z();
            this_pose_stamped.pose.orientation.w = qua.w();
            
            // toEulerAngle(qua,roll,pitch,yaw);
            // cout<<roll<<" "<<pitch<<" "<<yaw<<endl;
            
            // RationMatrixtoEulerAngle(R,roll,pitch,yaw);
            // cout<<roll<<" "<<pitch<<" "<<yaw<<endl;

            // cout<<"yaw "<<atan2(R(1,0),R(0,0))<<endl;
            // cout<<"pitch "<<atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)))<<endl;
            // cout<<"roll "<<atan2(R(2,1),R(2,2))<<endl;

            Eigen::Vector3d euler = R.eulerAngles(2, 0, 1);         //yaw roll pitch
            yaw=euler[0];roll=euler[1];pitch=euler[2];
            cout<<"yaw "<<euler[0]<<" roll "<<euler[1]<<" pitch "<<euler[2]<<endl;  
            
            fs<<cur_t<<" "<<x<<" "<<y<<" "<<z<<" "<<vx<<" "<<vy<<" "<<vz<<" "<<ax<< " "<<ay<<" "<<az<<" "<<euler[0]*180/M_PI<<" "<<euler[1]*180/M_PI<<" "<<euler[2]*180/M_PI<<endl;
            Eigen::Matrix3d R_yaw;
            Eigen::Matrix3d R_roll;
            Eigen::Matrix3d R_pitch;
            R_yaw<<cos(yaw),-sin(yaw),0,
                    sin(yaw),cos(yaw),0,
                    0   ,   0,  1;
            R_roll<<1,0,0,
                    0,cos(roll),-sin(roll),
                    0,sin(roll),cos(roll);
            R_pitch<<cos(pitch),0,sin(pitch),
                        0,1,0,
                        -sin(pitch),0,cos(pitch);
            R2=R_yaw*R_roll*R_pitch;
            cout<<"R"<<endl;
            cout<<R<<endl;
            cout<<"R2"<<endl;
            cout<<R2<<endl;
            
            

            //旋转矩阵
            R2<<cos(pitch)*cos(yaw)-sin(pitch)*sin(yaw)*sin(roll),  -sin(yaw)*cos(roll),    sin(pitch)*cos(yaw)+cos(pitch)*sin(yaw)*sin(roll),
                cos(pitch)*sin(yaw)+sin(pitch)*cos(yaw)*sin(roll),  cos(yaw)*cos(roll),     sin(pitch)*sin(yaw)-cos(pitch)*cos(yaw)*sin(roll),
                -cos(roll)*sin(pitch),                              sin(roll),              cos(pitch)*cos(roll);
            cout<<"R3"<<endl;
            cout<<R2<<endl; 
            cout<<"roll "<< asin(Y_B[2])<<endl;
            double sin_roll=R(2,1);
            double cos_roll=sqrt(1-sin_roll*sin_roll);
            double sin_pitch=-R(2,0)/cos_roll;
            cout<<"pitch "<< asin(sin_pitch)<<endl;

            // Eigen::Matrix3d R3;
            // R3.col(0)=Z_B;
            // R3.col(1)=X_B;
            // R3.col(2)=Y_B;
            // euler = R3.eulerAngles(1, 2, 0);         //yaw roll pitch
            // cout<<"yaw2 "<<euler[0]<<" roll2 "<<euler[1]<<" pitch2 "<<euler[2]<<endl;
            // R2=R_yaw*R_pitch*R_roll;
            // cout<<"R3"<<endl;
            // cout<<R3<<endl;
            // cout<<"R2"<<endl;
            // cout<<R2<<endl;
            // Eigen::Vector3d ZB_XC={-Z_B[2],0,Z_B[0]};
            // ZB_XC=ZB_XC/ZB_XC.norm();
            // Eigen::Vector3d YB_ZB;
            // double rolls=atan2(Z_B[0],Z_B[2]);
            // double temp=sqrt(pow(Z_B[2],2)+pow(Z_B[0],2));
            // YB_ZB[0]=-Z_B[0]*Z_B[1]/temp;
            // YB_ZB[1]=(pow(Z_B[0],2)+pow(Z_B[2],2))/temp;
            // YB_ZB[2]=-Z_B[1]*Z_B[2]/temp;
            //cout<<"ax "<<ax <<" ay "<<ay<<" az+g "<<az+g<<endl;
            // cout<<"rolls "<<rolls<<" "<<asin(Y_B[2])<<endl;
            // cout<<"ZB_XC "<<ZB_XC.transpose()<<endl;
            // cout<<"YB_ZB "<<YB_ZB.transpose()<<endl;
            // cout<<"X_B "<<R.col(0).transpose()<<endl;
            // cout<<"Y_B "<<R.col(1).transpose()<<endl;
            // cout<<"Z_B "<<R.col(2).transpose()<<endl;
            path.poses.push_back(this_pose_stamped);
            path_pub.publish(path);
            pose_pub.publish(this_pose_stamped);
            r.sleep();
        }
        
    }
    fs.close();
}