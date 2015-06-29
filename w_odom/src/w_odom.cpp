#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <cmath>

double PA = 0, PB = 0, delta_PA = 0, delta_PB = 0;
bool find = false, BA = false, BB = false;
int ncont = 0;
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double delta_x = 0;
double delta_y = 0;
double delta_th = 0;

double raio = 0;

ros::Publisher odom_pub;
geometry_msgs::Quaternion odom_quat;

double rad2deg(double rad) { return rad * (180/M_PI); }

bool near(double v1, double v2, double nearness){
  //  ROS_INFO("abs(%f - %f) [%f] <= %f", v1, v2, std::abs(v1-v2), nearness);
    return std::abs(v1 - v2) <= nearness;
}

void calcula_nova_odom(){

    BA = false;
    BB = false;

    delta_PA /= 1000; //Metros
    delta_PB /= 1000;

    //if(delta_PA == delta_PB)
    if(near(delta_PA, delta_PB, 0.001)) //tolerância = 1mm (andando em linha reta)
    {
        raio = 0;
        delta_th = 0;
    }
    else //Fazendo curva
    {

        if(near(std::abs(delta_PA), std::abs(delta_PB), 0.001)) //Curva sobre o próprio eixo
        {
            raio = 0;
            if(delta_PA > 0) delta_th = (std::abs(delta_PA) + std::abs(delta_PB))*1.73;
        }

        else{ //Curva com raio
            if(std::abs(delta_PA) > std::abs(delta_PB)){ //Casos 1 e 3
                if(delta_PA > 0)
                    raio = 0.297 * delta_PA / (delta_PA - delta_PB);
                else
                    raio = -0.297 * delta_PB / (delta_PB - delta_PA);
            }
            else{ //Casos 2 e 4
                if(delta_PA > 0)
                    raio = -0.297 * delta_PA / (delta_PA - delta_PB);
                else
                    raio = 0.297 * delta_PB / (delta_PB - delta_PA);
            }
        }

//        if(raio != 0) delta_th = delta_PA / raio; //Está em RAD.
        if(raio != 0) delta_th = (delta_PA + delta_PB)/ (2*raio); //Está em RAD.

    }

    th += delta_th;

    double vel = (delta_PA + delta_PB)/2;

    //delta_x = delta_PA * cos(th);
    //delta_y = delta_PA * sin(th);

    delta_x = vel * cos(th);
    delta_y = vel * sin(th);

    //delta_th = raio;

    x += delta_x;
    y += delta_y;

   // ROS_INFO("\n\ndelta_PA = %f, PA = %f, delta_PB = %f, PB = %f, raio = %f, th = %f, sen(th) = %f, cos(th) = %f",
   //          delta_PA, PA, delta_PB, PB, raio, th, sin(th), cos(th));

    //if(delta_PA != 0 && delta_PB != 0)
        //ROS_INFO("d_PA = %f, d_PB = %f, dx = %f, dy = %f, r = %f, x = %f, y = %f, th(deg) = %f", delta_PA, delta_PB, delta_x, delta_y, raio, x, y, rad2deg(th));
}
    //13500 - 1cm
    //1350 - 1mm

void Bridge_A_Callback(const std_msgs::Int32& msg)
{
   // ROS_INFO("Bridge A Callback: %d, PA = %f, delta_pa = %f", msg.data, PA, delta_PA);

    if(!find){
        PA = msg.data;
        delta_PA = 0;
        ncont++;
        if(ncont > 10)
            find = true;
    }
    else{
        if(abs(msg.data - PA) > 1350 && !BA){  //Andou 1mm
            delta_PA = (msg.data - PA)/1350;
            PA = msg.data;
            BA = true;

            if(BA && BB) calcula_nova_odom();
        }
    }
}

void Bridge_B_Callback(const std_msgs::Int32& msg)
{
   // ROS_INFO("Bridge B Callback: %d, PB = %f, delta_pb = %f", msg.data, PB, delta_PB);
    if(!find){

        //ROS_INFO("delta_PB = 0;");

        PB = msg.data;
        delta_PB = 0;
        ncont++;
        if(ncont > 10)
            find = true;
    }
    else{
        if(abs(msg.data - PB) > 1350 && !BB){  //Andou 1mm
            delta_PB = (msg.data - PB)/1350;
            PB = msg.data;
            BB = true;
            if(BA && BB) calcula_nova_odom();
        }
    }
   // ROS_INFO("Bridge B Callback: %d, PB = %f, delta_pb = %f", msg.data, PB, delta_PB);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ROS_INFO("W_Odom running!");

    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Rate r(10.0);

    sleep(5);

    ROS_INFO("W_ODOM: Aguardando posicao da Bridge");

    ros::Subscriber s1 = n.subscribe("/bridge/output/position/m1", 500, Bridge_A_Callback);
    ros::Subscriber s2 = n.subscribe("/bridge/output/position/m2", 500, Bridge_B_Callback);

    while(n.ok() && !find)
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("\n#_#_#_#_#_#_#_#_#_#_#_#_#_#_#_# W_ODOM: Posicao inicial encontrada: %f e %f #_#_#_#_#_#_#_#_#_#_#_#_#_#_#_#", PA, PB);

    tf::TransformBroadcaster odom_broadcaster;

    ROS_INFO("ok!");
    while(n.ok())
    {

        ros::spinOnce(); // check for incoming messages

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "/odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "/odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        r.sleep();
    }
}
