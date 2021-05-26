#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>  //ntohl

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>



#define PORT 8080
#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

int send_sock_, server_fd_, recv_sock_, udp_server_fd_, udp_send_fd_;
ros::Subscriber swarm_trajs_sub_, odoms_sub_, trigger_stop_sub, one_traj_sub_,trigger_origin_sub,trigger_goal_sub;
ros::Publisher  emergency_stop_pub_,traj_start_trigger_pub;
string tcp_ip_, udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
char send_buf_[BUF_LEN], recv_buf_[BUF_LEN], udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
nav_msgs::OdometryPtr odom_msg_;
nav_msgs::Odometry all_odom_[21];
nav_msgs::Odometry my_odom_;

std_msgs::EmptyPtr stop_msg_;

ros::Time t_last_odom;
ros::Time t_last_trigger;
double formation_scale = 1.0; 
double goal[21][2] = {{0,0}  ,{20,0},
                         {16,-2},{16,2},
                        {12,-4},{12,0},{12,4},                       
                       {8,-6},{8,-2},{8,2},{8,6},
                    {4,-8},{4,-4},{4,0},{4,4},{4,8},
                    {0,-8},{0,-4},{0,0},{0,4},{0,8}
                      };

bool can_fly_flag;
enum MESSAGE_TYPE
{
  ODOM = 888,
  TRIGGER,
  MULTI_TRAJ,
  ONE_TRAJ,
  STOP
} massage_type_;


int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    ROS_ERROR("[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}


int serializeOdom(const nav_msgs::OdometryPtr &msg)
{
  char *ptr = udp_send_buf_;

  unsigned long total_len = 0;
  total_len = sizeof(size_t) +
              msg->child_frame_id.length() * sizeof(char) +
              sizeof(size_t) +
              msg->header.frame_id.length() * sizeof(char) +
              sizeof(uint32_t) +
              sizeof(double) +
              7 * sizeof(double) +
              36 * sizeof(double) +
              6 * sizeof(double) +
              36 * sizeof(double);

  if (total_len + 1 > BUF_LEN)
  {
    ROS_ERROR("[bridge_node] Topic is too large, please enlarge BUF_LEN");
    return -1;
  }

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::ODOM;
  ptr += sizeof(MESSAGE_TYPE);

  // child_frame_id
  size_t len = msg->child_frame_id.length();
  *((size_t *)ptr) = len;
  ptr += sizeof(size_t);
  memcpy((void *)ptr, (void *)msg->child_frame_id.c_str(), len * sizeof(char));
  ptr += len * sizeof(char);

  // header
  len = msg->header.frame_id.length();
  *((size_t *)ptr) = len;
  ptr += sizeof(size_t);
  memcpy((void *)ptr, (void *)msg->header.frame_id.c_str(), len * sizeof(char));
  ptr += len * sizeof(char);
  *((uint32_t *)ptr) = msg->header.seq;
  ptr += sizeof(uint32_t);
  *((double *)ptr) = msg->header.stamp.toSec();
  ptr += sizeof(double);

  *((double *)ptr) = msg->pose.pose.position.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.position.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.position.z;
  ptr += sizeof(double);

  *((double *)ptr) = msg->pose.pose.orientation.w;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.orientation.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.orientation.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.orientation.z;
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    *((double *)ptr) = msg->pose.covariance[j];
    ptr += sizeof(double);
  }

  *((double *)ptr) = msg->twist.twist.linear.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.linear.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.linear.z;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.angular.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.angular.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.angular.z;
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    *((double *)ptr) = msg->twist.covariance[j];
    ptr += sizeof(double);
  }

  return ptr - udp_send_buf_;
}

int serializeStop(const std_msgs::EmptyPtr &msg)
{
  char *ptr = udp_send_buf_;

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::STOP;
  ptr += sizeof(MESSAGE_TYPE);

  return ptr - udp_send_buf_;
}

int deserializeStop()
{
  char *ptr = udp_recv_buf_;
  ptr += sizeof(MESSAGE_TYPE);

  return ptr - udp_recv_buf_;
}

int serializeTrigger(double x,double y,double z)
{
  char *ptr = udp_send_buf_;

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::TRIGGER;
  ptr += sizeof(MESSAGE_TYPE);

  *((double *)ptr) = x;
  ptr += sizeof(double);
  *((double *)ptr) = y;
  ptr += sizeof(double);
  *((double *)ptr) = z;
  ptr += sizeof(double);
  *((double *)ptr) = formation_scale;
  ptr += sizeof(double);

  return ptr - udp_send_buf_;
}

int deserializeTrigger(double& x,double& y,double& z)
{
  char *ptr = udp_recv_buf_;
  ptr += sizeof(MESSAGE_TYPE);

  x = *((double *)ptr);
  ptr += sizeof(double);
  y = *((double *)ptr);
  ptr += sizeof(double);
  z = *((double *)ptr);
  ptr += sizeof(double);
  formation_scale = *((double *)ptr);
  ptr += sizeof(double);

  return ptr - udp_recv_buf_;
}

int deserializeOdom(nav_msgs::OdometryPtr &msg)
{
  char *ptr = udp_recv_buf_;

  ptr += sizeof(MESSAGE_TYPE);

  // child_frame_id
  size_t len = *((size_t *)ptr);
  ptr += sizeof(size_t);
  msg->child_frame_id.assign((const char *)ptr, len);
  ptr += len * sizeof(char);

  // header
  len = *((size_t *)ptr);
  ptr += sizeof(size_t);
  msg->header.frame_id.assign((const char *)ptr, len);
  ptr += len * sizeof(char);
  msg->header.seq = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);
  msg->header.stamp.fromSec(*((double *)ptr));
  ptr += sizeof(double);

  msg->pose.pose.position.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.position.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.position.z = *((double *)ptr);
  ptr += sizeof(double);

  msg->pose.pose.orientation.w = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.orientation.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.orientation.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.orientation.z = *((double *)ptr);
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    msg->pose.covariance[j] = *((double *)ptr);
    ptr += sizeof(double);
  }

  msg->twist.twist.linear.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.linear.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.linear.z = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.angular.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.angular.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.angular.z = *((double *)ptr);
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    msg->twist.covariance[j] = *((double *)ptr);
    ptr += sizeof(double);
  }

  return ptr - udp_recv_buf_;
}



void odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg)
{
  ros::Time t_now = ros::Time::now();
  my_odom_ = *msg;
  if ((t_now - t_last_odom).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last_odom = t_now;

  msg->child_frame_id = std::to_string(drone_id_);

  int len = serializeOdom(msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (1)!!!");
  }
}

void emergency_stop_sub_udp_cb(const std_msgs::EmptyPtr &msg)
{
  int len = serializeStop(msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (2)!!!");
  }
}

void trigger_udp(double x,double y,double z)
{
  if(can_fly_flag==false){
      ROS_ERROR("can not fly!!!");
     return ;
  }
  ROS_ERROR("UDP send!!!");
  int len = serializeTrigger(x,y,z);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (2)!!!");
  }
} 
void trigger_origin_sub_udp_cb(const geometry_msgs::PoseStampedPtr &msg)
{
    trigger_udp(0.0,0.0,1);
}
void trigger_goal_sub_udp_cb(const geometry_msgs::PoseStampedPtr &msg)
{
    trigger_udp(50.0,0.0,1);
}

void udp_recv_fun()
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len;

  // Connect
  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    ROS_ERROR("[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
    {
      perror("recvfrom error:");
      exit(EXIT_FAILURE);
    }
    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {
    case MESSAGE_TYPE::STOP:
    {
        if (valread == deserializeStop())
        {
          //emergency_stop_pub_.publish(*stop_msg_);
          ROS_ERROR("Received stop_msg message" );
          nav_msgs::Path waypoints;
          geometry_msgs::PoseStamped pt;
          pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
          
          pt.pose.position.x = my_odom_.pose.pose.position.x + 1;
          pt.pose.position.y = my_odom_.pose.pose.position.y;
          pt.pose.position.z =  1;
          waypoints.poses.push_back(pt); 
          traj_start_trigger_pub.publish(waypoints);
        }
        else
        {
          ROS_ERROR("Received message length not matches the sent one (1)!!!");
          continue;
        }

      break;
    }

    case MESSAGE_TYPE::ODOM:
    {
      if(drone_id_ == 0){
        if (valread == deserializeOdom(odom_msg_))
        {
          all_odom_[atoi(odom_msg_->child_frame_id.c_str())] = *odom_msg_;
          cout<<odom_msg_->child_frame_id.c_str()<<endl;
        }
        else
        {
          ROS_ERROR("Received message length not matches the sent one (2)!!!");
          continue;
        }
      }

      break;
    }

    case MESSAGE_TYPE::TRIGGER:
    {
      double tmp_x,tmp_y,tmp_z;
      if (valread == deserializeTrigger(tmp_x,tmp_y,tmp_z))
      {
        ros::Time t_now = ros::Time::now();
        if((t_now - t_last_trigger).toSec() > 0.05){
          nav_msgs::Path waypoints;
          geometry_msgs::PoseStamped pt;
          pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
          
          pt.pose.position.x = goal[drone_id_][0]*formation_scale+tmp_x;
          pt.pose.position.y = goal[drone_id_][1]*formation_scale+tmp_y;
          pt.pose.position.z =  tmp_z;
          waypoints.poses.push_back(pt); 
          traj_start_trigger_pub.publish(waypoints);
          t_last_trigger = t_now;
        }
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (2)!!!");
        continue;
      }
      break;
    }

    default:
      
      ROS_ERROR("Unknown received message???");
      break;
    }
  }
}

int get_id() {
    int id = -1;
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            if(addressBuffer[0]=='1'&&addressBuffer[1]=='9'&&addressBuffer[2]=='2'){
            id = 10*(addressBuffer[11]-'0')+addressBuffer[12]-'0';
            printf("robot id = %d\n",id);
            break;
          }
        }
      }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return id;
}


void check(){
  if(drone_id_ != 0) return;
  bool flag = true;
  ros::Time t_now = ros::Time::now();
  for(int i=1;i<=20;++i){
    if((t_now - all_odom_[i].header.stamp).toSec() > 1){
      cout<<"drone "<< i << "doesn't work!!!!" <<endl;
      flag = false;
    }
  }
  can_fly_flag = flag;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosmsg_tcp_bridge");
  ros::NodeHandle nh("~");

  nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("odom_max_freq", odom_broadcast_freq_, 1.0);
  nh.param("formation_scale", formation_scale, 1.0);
  
  drone_id_ = get_id();
  odom_msg_.reset(new nav_msgs::Odometry);
  stop_msg_.reset(new std_msgs::Empty);

  t_last_odom = ros::Time::now();
  t_last_trigger = ros::Time::now();

  can_fly_flag = false;
  if (drone_id_ == -1)
  {
    ROS_ERROR("Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  if ( drone_id_ >= 1 )
  {
    traj_start_trigger_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints",10);
  }
  if(drone_id_ == 0)
  {
    trigger_origin_sub = nh.subscribe("/origin_trigger", 10, trigger_origin_sub_udp_cb);
    trigger_goal_sub = nh.subscribe("/goal_trigger", 10, trigger_goal_sub_udp_cb);
    trigger_stop_sub = nh.subscribe("/stop_trigger",10,emergency_stop_sub_udp_cb);
  }

  odoms_sub_ = nh.subscribe("my_odom", 10, odom_sub_udp_cb, ros::TransportHints().tcpNoDelay());

  boost::thread udp_recv_thd(udp_recv_fun);
  udp_recv_thd.detach();
  ros::Duration(0.1).sleep();

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  cout << "[rosmsg_tcp_bridge] start running" << endl;

  ros::Rate rate(100);
  bool status = ros::ok();
  while(status) 
  {
    ros::spinOnce();  
    check();    
    status = ros::ok();
    rate.sleep();
  }
  close(udp_server_fd_);
  close(udp_send_fd_);

  return 0;
}
