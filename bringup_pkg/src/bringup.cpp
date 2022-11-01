#include "robot.h"
#include "mbot_linux_serial.h"
#include <tf/transform_listener.h>
//test send value
double left_v_set=0;
double right_v_set=0;
unsigned char testSend=0x01;

//test receive value
int left_v=0;
int right_v=0;
float yaw=0.0;
int flag =0;
void callBack(const geometry_msgs::Twist& msg)
{
    left_v_set=msg.linear.x-(msg.angular.z*0.102); //   m/s
    right_v_set=msg.linear.x+(msg.angular.z*0.102);
    writeSpeed(left_v_set*1000,right_v_set*1000,testSend);//  mm/s
    ROS_INFO("%f,%f\n",left_v_set,right_v_set);
    
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"robot_bringup");
    ros::NodeHandle nh;
    
    
    robot::robot myrobot;
    if(!myrobot.init())
        ROS_ERROR("init failed");
    ROS_INFO("init successful");
    ros::Subscriber vel_listener=nh.subscribe("/cmd_vel",2,callBack);
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
    
        ROS_INFO("1");
        ros::spinOnce();
        ROS_INFO("2");
        
        readSpeed(yaw,left_v,right_v,flag); // mm/s
        ROS_INFO("3");
        
        ROS_INFO("yaw:%.3f,left_speed:%dmm/s,right_speed:%dmm/s\n,flag=%d",yaw,left_v,right_v,flag);
        float leftv = left_v/1000.0;
        float rightv = right_v/1000.0;
        float yaw_ = yaw; 
        myrobot.deal(leftv,rightv,yaw_);// m/s
        ROS_INFO("4");
        
        loop_rate.sleep();
        ROS_INFO("5");
        
    }
    return 0;
}
