#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <fstream>

using namespace std;

double vx = 0.0;
//double vy = 0.0;
double vth = 0.0, fator = 10000;

double max_vx = 0.1, max_vth = 0.1;

bool kill = false;

void limits_Callback(const geometry_msgs::Point& msg){
    max_vx = msg.x;
    max_vth = msg.z;
    ROS_INFO("new max_vx: %f", max_vx);
    ROS_INFO("new max_vth: %f", max_vth);
}

void kill_Callback(const std_msgs::Bool& msg)
{
    kill = msg.data;

    if(kill)ROS_INFO("base pre controller killed!");
    else ROS_INFO("base pre controller LIVING!");
}

void factor_Callback(const std_msgs::Float64& msg)
{
    fator = msg.data;

    ROS_INFO("Novo fator de multiplicacao: %l", fator);
}

void cmd_vel_Callback(const geometry_msgs::Twist& msg)
{
    //vx = msg.linear.x > max_vx ? max_vx : msg.linear.x;
    //vth = msg.angular.z  > max_vth ? max_vth : msg.angular.z;
   // vth = msg.angular.z + msg.linear.y/4; //aplicar uma pequena influencia de y sobre th



    //vx = msg.linear.x;
    //vth = msg.angular.z;

    if(msg.linear.x  >= 0){
        vx = msg.linear.x > max_vx ? max_vx : msg.linear.x;
    }
    else{
        vx = msg.linear.x < -max_vx ? -max_vx : msg.linear.x;
    }

    if(msg.angular.z  >= 0){
        vth = msg.angular.z > max_vth ? max_vth : msg.angular.z;
    }
    else{
        vth = msg.angular.z < -max_vth ? -max_vth : msg.angular.z;
    }

    //vth = -vth;

    //if(vth != msg.angular.z + msg.linear.y/4) ROS_INFO("Limitando Vth de %f para %f",msg.angular.z + msg.linear.y/4, vth);
    //if(vx != msg.linear.x) ROS_INFO("Limitando Vx de %f para %f",msg.linear.x , vx);
}

bool near_zero(double val, double nearness)
{
    return abs(val) < nearness;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_simple_pre_controller");
    ROS_INFO("base_simple_pre_controller running!");

    ros::NodeHandle n;
    std_msgs::Int32 motor1, motor2;
    double vr1, vr2;

    ros::Subscriber s1 = n.subscribe("/cmd_vel", 500, cmd_vel_Callback);

    ros::Subscriber s2 = n.subscribe("/kill", 500, kill_Callback);

    ros::Subscriber s3 = n.subscribe("/base_pre_controller/limits", 500, limits_Callback);
    n.subscribe("/base_pre_controller/factor", 500, factor_Callback);

    ros::Publisher p1 = n.advertise<std_msgs::Int32>("/bridge/input/speed/m1", 500);
    ros::Publisher p2 = n.advertise<std_msgs::Int32>("/bridge/input/speed/m2", 500);

    /*ifstream fin("fator.txt");
    fin >> fator;
    fin.close();*/

    ROS_INFO("Fator de multiplicacao: %f", fator);
    ROS_INFO("max_vx: %f", max_vx);
    ROS_INFO("max_vth: %f", max_vth);

    ros::Rate r(10.0);

    while(n.ok())
    {

        ros::spinOnce(); // check for incoming messages

        if(near_zero(vth, 0.001))
        {
            vr1 = vr2 = vx;
        }
        else if(near_zero(vx, 0.001)){
            vr1 = vth;
            vr2 = -vth;
        }
        else {
            vr1 = vx + vth/2;
            vr2 = vx - vth/2;
        }

        if(kill) fator = 0;

        motor1.data = vr1 * fator;
        motor2.data = vr2 * fator;

        p1.publish(motor1);
        p2.publish(motor2);

        r.sleep();
    }
}
