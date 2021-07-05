#include "bezier_traj.h"
double calculatePos(double *coeff,double time_slot,double t_T)
{
    double p= coeff[0]*pow(1-t_T,6)+6*coeff[1]*t_T*pow(1-t_T,5)+15*coeff[2]*pow(t_T,2)*pow(1-t_T,4)+20*coeff[3]*pow(t_T,3)*pow(1-t_T,3)+
            15*coeff[4]*pow(t_T,4)*pow(1-t_T,2)+6*coeff[5]*pow(t_T,5)*pow(1-t_T,1)+coeff[6]*pow(t_T,6);
    p=p*time_slot;
    return p;
}
double calculateVelocity(double *coeff,double time_slot,double t_T)
{
    double p= (coeff[1]-coeff[0])*pow(1-t_T,5)+5*(coeff[2]-coeff[1])*pow(t_T,1)*pow(1-t_T,4)+10*(coeff[3]-coeff[2])*pow(t_T,2)*pow(1-t_T,3)
              +10*(coeff[4]-coeff[3])*pow(t_T,3)*pow(1-t_T,2)+5*(coeff[5]-coeff[4])*pow(t_T,4)*pow(1-t_T,1)+(coeff[6]-coeff[5])*pow(t_T,6);
    p=p*6;
    return p;
}
double calculateAcc(double *coeff,double time_slot,double t_T)
{
  double p= (coeff[2]-2*coeff[1]+coeff[0])*pow(1-t_T,4)+4*(coeff[3]-2*coeff[2]+coeff[1])*pow(t_T,1)*pow(1-t_T,3)+6*(coeff[4]-2*coeff[3]+coeff[2])*pow(t_T,2)*pow(1-t_T,2)
           +4*(coeff[5]-2*coeff[4]+coeff[3])*pow(t_T,3)*pow(1-t_T,1)+(coeff[6]-2*coeff[5]+coeff[4])*pow(t_T,4);
  p=p*30/time_slot;
  return p; 
}
int main(int argc ,char **argv){
  
    //参数初始化
    //double a[10000][10000]={1};
    ros::init (argc, argv, "bezier_minimumjerk_node");
    ros::NodeHandle ph;
    ros::Rate r(10);
    ofstream fs;
    fs.open("/home/dji/catkin_ws/src/ooqp_study/date/bezier_minimum_data.txt");

    

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
    std::vector<waypoint_info> w;
    std::vector<traj_info> traj;
    std::vector<std::vector <double> > coeff;
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
            std::vector<double> x(6);
        }
        if(i>=1)
        {
            traj_info t;
            t.start_point=w[i-1];
            t.end_point=w[i];
            traj.push_back(t);
        }
    }

    //flag 判断是否无解
    int flag=0;
    bezier_traj bez_Tra;
    bez_Tra.waypoints_bezier_traj(w,traj,flag);
    if(flag==0)
    {
      cout<<" flag=0 No Sloution!!!"<<endl;
      return 0;
    }
    
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("bezier_path", 1);
    ros::Publisher pose_pub = ph.advertise<geometry_msgs::PoseStamped>("bezier_poses", 1);
    ros::Publisher marker_pub = ph.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
    nav_msgs::Path path;
    path.header.frame_id="/world";
    cin>>flag;
    for(int i=0;i<w.size();i++)
    {
        double x= calculatePos(traj[i].x_ctr_point,time_slots[i],0/time_slots[i]);
        double y= calculatePos(traj[i].y_ctr_point,time_slots[i],0/time_slots[i]);
        double z= calculatePos(traj[i].z_ctr_point,time_slots[i],0/time_slots[i]);
        if(i==w.size()-1)
        {
          x= calculatePos(traj[i-1].x_ctr_point,time_slots[i-1],1);
          y= calculatePos(traj[i-1].y_ctr_point,time_slots[i-1],1);
          z= calculatePos(traj[i-1].z_ctr_point,time_slots[i-1],1);
          i++;
        }
        
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
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // 设置标记的比例，所有方向上尺度1表示1米
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
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
    for(int j=0;j<(waypoint_num-1);j++)
    {
       
        for(double i=0;i<time_slots[j];i=i+0.05)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            double x= calculatePos(traj[j].x_ctr_point,time_slots[j],i/time_slots[j]);
            double y= calculatePos(traj[j].y_ctr_point,time_slots[j],i/time_slots[j]);
            double z= calculatePos(traj[j].z_ctr_point,time_slots[j],i/time_slots[j]);
            double vx=calculateVelocity(traj[j].x_ctr_point,time_slots[j],i/time_slots[j]);
            double vy=calculateVelocity(traj[j].y_ctr_point,time_slots[j],i/time_slots[j]);
            double vz=calculateVelocity(traj[j].z_ctr_point,time_slots[j],i/time_slots[j]);
            double ax=calculateAcc(traj[j].x_ctr_point,time_slots[j],i/time_slots[j]);
            double ay=calculateAcc(traj[j].y_ctr_point,time_slots[j],i/time_slots[j]);
            double az=calculateAcc(traj[j].z_ctr_point,time_slots[j],i/time_slots[j]);
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
            path.poses.push_back(this_pose_stamped);
            path_pub.publish(path);
            pose_pub.publish(this_pose_stamped);
            r.sleep();
        }
        
    }
}