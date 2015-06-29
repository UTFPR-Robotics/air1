#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher p ;
sensor_msgs::LaserScan hok;
bool key = true;

void hokuyoCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    //ROS_INFO("new HOKUYO message!!");
    if(key){
        //ROS_INFO("messaged forwarded!!");
        hok = *scan_in;
        key = false;
    }

}

int main (int argc, char **argv)
{
    int c;
    ros::init(argc, argv, "hokuyo2pointCloud");

    ros::NodeHandle n;
    ROS_INFO("hokuyo2pointCloud running!");

    ros::Rate loop_rate(60);

    p = n.advertise<sensor_msgs::PointCloud2>("/narrow_stereo/points_filtered2", 100);

    ros::Subscriber mr = n.subscribe("/Hokuyo/scan", 1000, hokuyoCallback);

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 cloud2;
    ros::Time now;

    while(ros::ok()){
        ros::spinOnce();
        if(!key)
        {
            now = ros::Time::now();
          //  ROS_INFO("Waiting for TF!!");
            //if(listener_.waitForTransform("laser", "/base_link", hok.header.stamp + ros::Duration().fromSec(hok.ranges.size()*hok.time_increment), ros::Duration(1.0)))
            if(listener_.waitForTransform("laser", "/base_link", now, ros::Duration(3.0)))
                {
          //          ROS_INFO("We have a transform!!");
                    projector_.transformLaserScanToPointCloud("/base_link", hok, cloud, listener_);

            //        ROS_INFO("PC -> PC2");
	                sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

              //      ROS_INFO("Publish");
                    p.publish(cloud2);
                    key = true;
                }
        }
        loop_rate.sleep();
    }
}
