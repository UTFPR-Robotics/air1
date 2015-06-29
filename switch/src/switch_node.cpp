#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

double vx = 0.0, vy = 0.0, vth = 0.0;
bool manobra_done = false, movebase_done = false;
enum switch_options {nav, manobra} the_switch;
ifstream myfile;

ros::Publisher cmdvel, nav_goal, manobra_goal;
geometry_msgs::Twist t;


/**     rosmsg show geometry_msgs/PoseStamped

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w

*/

void send_nav_goal(geometry_msgs::Twist next_goal)
{

    geometry_msgs::PoseStamped nav_goal_msg;

    nav_goal_msg.header.stamp = ros::Time::now();
    nav_goal_msg.header.frame_id = "map";

    nav_goal_msg.pose.position.x = next_goal.linear.x;
    nav_goal_msg.pose.position.y = next_goal.linear.y;

    tf::Quaternion q = tf::createQuaternionFromYaw(next_goal.angular.z*M_PI/180);

    nav_goal_msg.pose.orientation.x = q.x();
    nav_goal_msg.pose.orientation.y = q.y();
    nav_goal_msg.pose.orientation.z = q.z();
    nav_goal_msg.pose.orientation.w = q.w();

    cout << "Publicando goal - nav" << endl;
    nav_goal.publish(nav_goal_msg);
    //ros::spinOnce();
    cout << "Publicado!" << endl;
    the_switch = nav;
    //ros::spin();
}
void send_manobra_goal(geometry_msgs::Twist next_goal)
{

    next_goal.angular.x = 0;
    cout << "Publicando goal - manobra" << endl;
    manobra_goal.publish(next_goal);
    //ros::spinOnce();
    cout << "Publicado!" << endl;
    the_switch = manobra;
    //ros::spin();
}

geometry_msgs::Twist read_next_goal()
{
    float x, y, th, swt;
    geometry_msgs::Twist ret;

    myfile >> x >> y >> th >> swt;

    if(myfile.eof())
    {
        cout << "fim do arquivo! "<< endl; // Recomecando..." << endl;
        myfile.close();
        exit(0);
        //myfile.open("/home/robo/catkin_ws/src/robo/launch/goals.txt");
        //myfile >> x >> y >> th >> swt;
    }

    cout << "Proximo goal: \nx: " << x << "| y: " << y << "| th: " << th*M_PI/180 <<" (" << th << ") | swt: " << swt << " (nav = " << nav << ", manobra = " << manobra << ") " << endl;

    ret.linear.x = x;
    ret.linear.y = y;
    ret.angular.z = th;

    ret.angular.x = swt;

    return(ret);

}

/**    rosmsg show geometry_msgs/Twist
    geometry_msgs/Vector3 linear
      float64 x --------> goal.x
      float64 y --------> goal.y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x --------> switch
      float64 y
      float64 z --------> goal.theta
*/

void core(void)
{

    while(manobra_goal.getNumSubscribers() == 0);
    while(nav_goal.getNumSubscribers() == 0);

    geometry_msgs::Twist next_goal = read_next_goal();

    sleep(1);

    switch((int)next_goal.angular.x)
    {
    case 1:
        send_nav_goal(next_goal);
        break;
    case 0:
        send_manobra_goal(next_goal);
        break;
    default:
        cout << "Switch found an error finding out which controller it should use. Halting." << endl;
        exit(1);
        break;
    }
    cout << "core end" << endl;
}

void cmd_vel_Callback(const geometry_msgs::Twist& msg)
{
    if(the_switch == nav)
    {
        t.linear.x = msg.linear.x;
        t.linear.y = msg.linear.y;
        t.angular.z = msg.angular.z;
        cmdvel.publish(t);
        //ros::spin();
    }
}

void manobra_vel_Callback(const geometry_msgs::Twist& msg)
{
    if(the_switch == manobra)
    {
        t.linear.x = msg.linear.x;
        t.linear.y = msg.linear.y;
        t.angular.z = msg.angular.z;
        cmdvel.publish(t);
        //ros::spin();
    }
}

void manobra_done_Callback(const std_msgs::Bool& msg)
{
    manobra_done = msg.data;
    core();
}

void movebase_done_Callback(const move_base_msgs::MoveBaseActionResult& msg)
{
    if(msg.status.status == 3){
        movebase_done = true;
        core();
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "switch_node");
    ros::NodeHandle n;
    cout << "Switch running!" << endl;

    cmdvel = n.advertise<geometry_msgs::Twist>("cmd_vel_output", 100);
    nav_goal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    manobra_goal = n.advertise<geometry_msgs::Twist>("manobra/manobra_goal", 100);

    ros::Subscriber s1 = n.subscribe("/cmd_vel", 500, cmd_vel_Callback);
    ros::Subscriber s2 = n.subscribe("manobra/manobra_vel", 500, manobra_vel_Callback);
    ros::Subscriber s3 = n.subscribe("manobra/manobra_done", 500, manobra_done_Callback);
    ros::Subscriber s4 = n.subscribe("move_base/result", 500, movebase_done_Callback);

    myfile.open("/home/robo/catkin_ws/src/robo/launch/switch_goals.txt");

    if (myfile.is_open())
    {
        cout << "Arquivo aberto!" << endl;
        core();
    }

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    printf("Switch fim\n");
    exit(0);
}

