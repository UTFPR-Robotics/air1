#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

enum goal_types {angular, linear} goal_type;

ros::Publisher manobra_done, manobra_vel;
//bool new_goal = false;
geometry_msgs::Twist goal;
geometry_msgs::PoseStamped pose_atual;
float yaw = 0, aux_yaw = 0;
bool goal_on = false, goal_end = false;
float erro = 0, speed_lim_x = 0, speed_lim_th = 0, ganho_x = 0, ganho_th = 0, velocidade_atual_x = 0, velocidade_atual_th = 0, lim_accel_x = 0, lim_accel_th = 0, erro_max_x = 0, erro_max_th = 0;
int spin = 0;
bool lock_spin_up = false, lock_spin_down = false;

void publish_speeds()
{
    geometry_msgs::Twist msg;

    msg.linear.x = velocidade_atual_x;
    msg.angular.z = velocidade_atual_th;

    manobra_vel.publish(msg);
}

void get_ganhos()
{
    ifstream myfile;

    myfile.open("/home/robo/catkin_ws/src/robo/launch/ganhos.txt");
    if (myfile.is_open())
    {
        cout << "Arquivo ganhos aberto" << endl;
        myfile >> ganho_x >> ganho_th;
        myfile.close();
    }
    cout << "ganhos: x = " << ganho_x << ", th = " << ganho_th << endl;
}

void get_erro_max()
{
    ifstream myfile;

    myfile.open("/home/robo/catkin_ws/src/robo/launch/erro_max.txt");
    if (myfile.is_open())
    {
        cout << "Arquivo erro_max aberto" << endl;
        myfile >> erro_max_x >> erro_max_th;
        myfile.close();
    }
    cout << "erro_max: x = " << erro_max_x << ", th = " << erro_max_th << endl;
}

void get_speedlimits()
{
    ifstream myfile;

    myfile.open("/home/robo/catkin_ws/src/robo/launch/manobra_speed_lim.txt");
    if (myfile.is_open())
    {
        cout << "Arquivo manobra_speed_lim aberto" << endl;
        myfile >> speed_lim_x >> speed_lim_th;
        myfile.close();
    }
    cout << "speed limits: x = " << speed_lim_x << ", th = " << speed_lim_th << endl;
}

void get_accel_limits()
{
    ifstream myfile;

    myfile.open("/home/robo/catkin_ws/src/robo/launch/limite_accel.txt");
    if (myfile.is_open())
    {
        cout << "Arquivo limite_accel aberto" << endl;
        myfile >> lim_accel_x >> lim_accel_th;
        myfile.close();
    }
    cout << "accel limits: x = " << lim_accel_x << ", th = " << lim_accel_th << endl;
}

goal_types get_goal_type(void)
{
    cout << " linear (" << goal.linear.x << ") - pose_atual_x (" << pose_atual.pose.position.x << ") = " << abs(goal.linear.x - pose_atual.pose.position.x) << endl;
    cout << " angular (" << goal.angular.z << ") - yaw (" << yaw << ") = " << abs(goal.angular.z - yaw ) << endl;

    //if (abs(goal.linear.x - pose_atual.pose.position.x) < 0.5)
    if (abs(goal.linear.x - pose_atual.pose.position.x) < abs(goal.angular.z - yaw))
    {
        cout << "goal angular" << endl;
        return angular;
    }
    cout << "goal linear" << endl;
    return linear;
}

void manobra_goal_Callback(const geometry_msgs::Twist& msg)
{
    goal.linear.x = msg.linear.x + pose_atual.pose.position.z;
    goal.linear.y = msg.linear.y + pose_atual.pose.position.y;
    goal.angular.z = msg.angular.z + yaw;

    goal_type = get_goal_type();

    velocidade_atual_x = 0;
    velocidade_atual_th = 0;

    publish_speeds();
    ros::spinOnce();
    sleep(1);
    goal_on = true;
    goal_end = false;
}

void odom_Callback(const nav_msgs::Odometry& msg)
{
    float q0, q1, q2, q3, last_yaw = aux_yaw;

    q0 = msg.pose.pose.orientation.x;
    q1 = msg.pose.pose.orientation.y;
    q2 = msg.pose.pose.orientation.z;
    q3 = msg.pose.pose.orientation.w;

    yaw = atan2 (2 * (q0*q1+q2*q3),(1 - 2 * ( q1*q1+q2*q2 ) ) ); //rotacao em z
    //yaw = asin (2 * (q0*q2-q3*q1)); //robo lateral

    /*if(yaw < 0)
        yaw += 2 * M_PI;

    if(yaw > M_PI)
        yaw = (2 * M_PI) - yaw;*/

    if(yaw < 0)
        yaw *= -1;
    else
        yaw = (2 * M_PI) - yaw;

    yaw *= 180 / M_PI;

    if(last_yaw - yaw > 50 && !lock_spin_up){
        lock_spin_up = true;
        lock_spin_down = false;
        spin ++;
    }

    if(last_yaw - yaw < -50 && !lock_spin_down){
        lock_spin_down = true;
        lock_spin_up = false;
        spin --;
    }

    if(yaw > 100 && yaw < 200 && lock_spin_up) lock_spin_up = false;
    if(yaw > 100 && yaw < 200 && lock_spin_down) lock_spin_down = false;

    aux_yaw = yaw;

    yaw += spin * 360;


    //ROS_INFO("SETTING current position: %f, %f, %f", msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
    pose_atual.pose.position = msg.pose.pose.position;
    pose_atual.pose.orientation = msg.pose.pose.orientation;
}

void stop ()
{
    cout << endl << "goal alcancado! " << endl;
    velocidade_atual_th = 0;
    velocidade_atual_x = 0;
    goal_end = true;
}

void testa_erro()
{
    switch(goal_type)
    {
    case angular:
        if(erro > 0 && erro < erro_max_th)
            stop();
        if(erro < 0 && erro > -erro_max_th)
            stop();
        break;
    case linear:
        if(erro > 0 && erro < erro_max_x)
            stop();
        if(erro < 0 && erro > -erro_max_x)
            stop();
        break;
    default:
        cout << "Erro na TestaErro! Undefined goal_type " << endl;
        exit(1);
        break;
    }
}

void processa_goal()
{
    float controle = 0;

    switch(goal_type)
    {
    case angular:

        erro = goal.angular.z - yaw;
        controle = erro * ganho_th;

        if(controle > 0)
        {
            if(controle > velocidade_atual_th && velocidade_atual_th < speed_lim_th)
            {
                velocidade_atual_th += lim_accel_th;
                break;
            }
            if(controle < velocidade_atual_th)
            {
                velocidade_atual_th = controle;
                break;
            }
        }

        if(controle < 0)
        {
            if(controle < velocidade_atual_th && velocidade_atual_th > -speed_lim_th)
            {
                velocidade_atual_th -= lim_accel_th;
                break;
            }
            if(controle > velocidade_atual_th)
            {
                velocidade_atual_th = controle;
                break;
            }
        }

        cout << "erro angular = " << goal.angular.z << " - " << yaw << " = " << erro << " | controle = " << erro << " * " << ganho_th << " = " << controle << "\r";

        break;
    case linear:
        erro = goal.linear.x - pose_atual.pose.position.x;
        controle = erro * ganho_x;

        if(controle > 0)
        {
            if(controle > velocidade_atual_x && velocidade_atual_x < speed_lim_x)
            {
                velocidade_atual_x += lim_accel_x;
                break;
            }
            if(controle < velocidade_atual_x)
            {
                velocidade_atual_x = controle;
                break;
            }
        }


        if(controle < 0)
        {
            if(controle < velocidade_atual_x && velocidade_atual_x > -speed_lim_x)
            {
                velocidade_atual_x -= lim_accel_x;
                break;
            }
            if(controle > velocidade_atual_x)
            {
                velocidade_atual_x = controle;
                break;
            }
        }

        cout << "erro linear = " << goal.linear.x << " - " << pose_atual.pose.position.x << " = " << erro << " | controle = " << erro << " * " << ganho_x << " = " << controle << "\r";

        /* cout << "erro angular = " << goal.angular.z << " - " << yaw << " = " << erro << endl
          << "controle = " << erro << " * " << ganho_th << " = " << controle << endl;*/

        //goal.linear.x - pose_atual.pose.position.x
        break;
    default:
        cout << "Erro na Processa Goal! Undefined goal_type " << endl;
        exit(1);
        break;
    }

    testa_erro();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "manobra_node");
    ros::NodeHandle n;
    cout << "Manobra running!" << endl;

    ///-------------- Publishers
    manobra_vel = n.advertise<geometry_msgs::Twist>("manobra/manobra_vel", 100);
    manobra_done = n.advertise<std_msgs::Bool>("manobra/manobra_done", 100);

    ///-------------- Subscribers

    ros::Subscriber s1 = n.subscribe("manobra/manobra_goal", 500, manobra_goal_Callback);
    ros::Subscriber s2 = n.subscribe("/odom", 500, odom_Callback);


    /* ifstream myfile;// ("/home/robo/catkin_ws/src/robo/launch/goals.txt");

     myfile.open("/home/robo/catkin_ws/src/robo/launch/manobra_speed_lim.txt");
     if (myfile.is_open()) cout << "Arquivo aberto" << endl;*/

    get_speedlimits();
    get_ganhos();
    get_erro_max();
    get_accel_limits();

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        if(goal_on){
            if(goal_end){
                std_msgs::Bool stop_msg;
                stop_msg.data = true;
                manobra_done.publish(stop_msg);
                goal_on = false;
            }
            else
            {
                processa_goal();
                publish_speeds();
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    printf("Manobra fim\n");
    exit(0);
}

