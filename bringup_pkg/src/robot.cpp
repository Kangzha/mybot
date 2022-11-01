#include "vector"
#include "robot.h"
#include "mbot_linux_serial.h"

namespace robot
{
    boost::array<double,36> odom_pose_covariance={
        {
            1e-9,0,0,0,0,0,
            0,1e-3,1e-9,0,0,0,
            0,0,1e6,0,0,0,
            0,0,0,1e6,0,0,
            0,0,0,0,1e6,0,
            0,0,0,0,0,1e-9
        }
    };
    boost::array<double,36> odom_twist_covariance={
        {
            1e-9,0,0,0,0,0,
            0,1e-3,1e-9,0,0,0,
            0,0,1e6,0,0,0,
            0,0,0,1e6,0,0,
            0,0,0,0,1e6,0,
            0,0,0,0,0,1e-9
        }
    };
    robot::robot():pos_x(0.0),pos_y(0.0),th(0.0),vx(0),vy(0),vth(0.0){}
    robot::~robot(){};

    bool robot::init()
    {
        serialInit();
        ros::Time::init();
        current_time=ros::Time::now();
        last_time=ros::Time::now();
        pub=nh.advertise<nav_msgs::Odometry>("odom",2);
        return true;
    }

    void robot::calculateOdom(float vx_,float vy_,float th_)
    {
        ros::Time cur_time;
        cur_time=ros::Time::now();
        float dt=(cur_time-last_time).toSec();
        float delta_x = (((vx_+vy_)/2)*cos(th_))*dt;  //rad
        float delta_y= (((vx_+vy_)/2)*sin(th_))*dt;

        robot::pos_x += delta_x;
        robot::pos_y += delta_y;

        last_time = cur_time;
    }
    
    void robot::publish_Odom_Tf()
    {
        ros::Time cur_time;
        cur_time=ros::Time::now();
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp=cur_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(robot::th);
        odom_trans.transform.translation.x=robot::pos_x;
        odom_trans.transform.translation.y=robot::pos_y;
        odom_trans.transform.translation.z=0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry msgl;
        msgl.header.stamp = cur_time;
        msgl.header.frame_id = "odom";
        msgl.pose.pose.position.x=robot::pos_x;
        msgl.pose.pose.position.y=robot::pos_y;
        msgl.pose.pose.position.z=0.0;
        msgl.pose.pose.orientation = odom_quat;
	msgl.pose.covariance = odom_pose_covariance;
        msgl.child_frame_id = "base_footprint";
        msgl.twist.twist.linear.x=(vx+vy)/2;
        msgl.twist.twist.linear.y=vy;
	msgl.twist.twist.angular.z=vth;
        msgl.twist.covariance= odom_twist_covariance;
        pub.publish(msgl);

    }
    bool robot::deal(float left_wheel_speed,float right_wheel_speed,float yaw_)
    {
        float th_= yaw_/57.3;  //  rad/s
        robot::th = th_;
        float vx_=left_wheel_speed;
        float vy_=right_wheel_speed;
        vth=(vy-vx)/0.204;   // l=20.4cm
        calculateOdom(vx_,vy_,th_);
        publish_Odom_Tf();
    }
}
