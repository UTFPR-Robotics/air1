#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

using namespace std;

ros::Publisher p;

void kinectCallback (const sensor_msgs::PointCloud2::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud2 m;
    m = *scan_in;
    p.publish(m);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "kin2cloud_in");

    ros::NodeHandle n;
    ROS_INFO("kin2cloud_in running!");

    p = n.advertise<sensor_msgs::PointCloud2>("cloud_in", 100);

    ros::Subscriber mr = n.subscribe("/camera/depth/points", 1000, kinectCallback);
    ros::spin();
}
