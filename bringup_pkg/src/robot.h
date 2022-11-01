#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "boost/asio.hpp"
#include "ddpg/robot_vel_msg.h"
#include "geometry_msgs/Twist.h"

namespace robot
{
    class robot
    {
        public:
            robot();
            ~robot();
            bool init();
            bool deal(float left_wheel_speed,float right_wheel_speed,float yaw_);
        private:
            void calculateOdom(float vx, float vy, float th);
            void publish_Odom_Tf();

        public:
            ros::Time current_time, last_time;
            float pos_x;
            float pos_y;
            float th;
            int vx;  //vx:left_wheel_speed
            int vy;
            float vth;     
            ros::NodeHandle nh;
            ros::Publisher pub;
            tf::TransformBroadcaster odom_broadcaster;
    };
}

