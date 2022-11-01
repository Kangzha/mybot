#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

extern void serialInit();
extern void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag);
//extern bool readSpeed(double &Left_v,double &Right_v,double &yaw,double &roll,double &pitch,double &x,double &y,double &z,unsigned char &ctrlFlag);
extern bool readSpeed(float &yaw,float &roll,float &pitch,float &accx,float &accy,float &accz,float &gyrox,float &gyroy,float &gyroz,float &mx,float &my,float &mz,short &flag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
