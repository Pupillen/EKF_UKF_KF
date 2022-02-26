/*** 
 * @Author: ztr
 * @Date: 2022-02-26 16:13:26
 * @LastEditTime: 2022-02-26 16:35:42
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /EKF_KF_UKF/test/src/collect_data.cpp
 */
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include "tf/transform_datatypes.h"//转换函数头文件
//https://www.codeleading.com/article/80104866205/
using namespace std;
normal_distribution<double>r(0,0.1);
random_device rd;
default_random_engine rng(rd());
nav_msgs::Odometry posts_pose;
geometry_msgs::Point point;
using namespace std;
ros::Subscriber odom_sub;

//高斯分布随机odom
/* void odom_call(const nav_msgs::OdometryConstPtr &odom_msg)
{posts_pose=*(odom_msg);                                                                                                         
posts_pose.pose.pose.position.x=odom_msg->pose.pose.position.x+r(rng);
posts_pose.pose.pose.position.y=odom_msg->pose.pose.position.y+r(rng);
posts_pub.publish(posts_pose);
} */

//sin,cos随机扰动Odom
//
//运动学模型
//需要收集l.x,l.y,yaw作为当前状态的输入u
//有噪声的p.x,p.y作为观测变量x
//计算ekf
using namespace std;
ofstream data_log;
normal_distribution<double>r(0,1);
random_device rd;
default_random_engine rng(rd());
int radius=0;
long long iteration=0;
double lx,ly,yaw,px,py,px_noise,py_noise;
geometry_msgs::Vector3 omg;
void odom_call(const nav_msgs::OdometryConstPtr &odom_msg)
{
iteration++;
static double theta=0;
px=odom_msg->pose.pose.position.x;
py=odom_msg->pose.pose.position.y;
px_noise=px+radius*cos(theta)+r(rng);
py_noise=py+radius*sin(theta)+r(rng);
lx=odom_msg->twist.twist.linear.x;
ly=odom_msg->twist.twist.linear.y;
theta+=M_PI/1000;
if(theta>2*M_PI+0.0001){theta-=2*M_PI;}
//计算yaw
tf::Quaternion quat;
tf::quaternionMsgToTF((*odom_msg).pose.pose.orientation,quat);
double roll, pitch;//定义存储r\p\y的容器
tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
data_log<<iteration<< "\t"<<lx<<"\t"<<ly<<"\t"<<yaw<<"\t"<<px<<"\t"<<py<<"\t"<<px_noise<<"\t"<<py_noise<<"\n";
}

int main(int argc,char* argv[])
{   ros::init(argc,argv,"imitation");
    ros::NodeHandle n;
    data_log.open("posts_data");
    odom_sub=n.subscribe("/odom",10,odom_call);
    n.param("radius",radius,1);
    ros::spin();
    data_log.close();
    return 0;
}