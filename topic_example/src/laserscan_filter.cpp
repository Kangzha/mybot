#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "topic_example/laserscan_filted.h"
#include <cmath>
#define RAD2DEG(x) ((x)*180./M_PI)

class LaserScanFilter
{
public:
  LaserScanFilter()
  {
    pub_ = n.advertise<topic_example::laserscan_filted>("scan_filted", 1000);

    sub_ = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, &LaserScanFilter::scanCallback,this);
  }
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    topic_example::laserscan_filted scan_filted;
    scan_filted.ranges.resize(24);
    //int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), 360);//count);
    ROS_INFO("angle_range, %f, %f,count:%d", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max),360);//count);  // -3.14  +3.14
    //grab 24 data into robot_scan
    int i=0;
    //for(int a = 0; a < count; a++) {
    for(int a = 0; a < 360; a++) {
	if(a==90 || a==97 ||  a==104 ||  a==111 ||  a==118 ||  a==126 ||  a==134 ||  a==142 ||  a==150 ||  a==158 ||  a==165 ||  a==173 ||  a==180 ||  a==187 ||  a==194 ||  a==202 ||  a==209 ||  a==217 ||  a==224 ||  a==231 ||  a==238 ||  a==245 ||  a==253 ||  a==261 )
        //if(a==0 || a==7 ||  a==14 ||  a==21 ||  a==28 ||  a==36 ||  a==44 ||  a==52 ||  a==60 ||  a==68 ||  a==76 ||  a==83 ||  a==90 ||  a==275 ||  a==283 ||  a==291 ||  
//a==298 ||  a==305 ||  a==313 ||  a==321 ||  a==329 ||  a==337 ||  a==345 ||  a==353 )
        {
           scan_filted.ranges[i]=scan->ranges[a]; 
           if(isinf(scan->ranges[a]))
           {
             scan_filted.ranges[i] = 8.0;
           }
           ++i; 
        }
} 
    pub_.publish(scan_filted);
  }

private:
   ros::NodeHandle n;
   ros::Publisher pub_;
   ros::Subscriber sub_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    LaserScanFilter obj;

    ros::spin(); 
    
    return 0;
}
