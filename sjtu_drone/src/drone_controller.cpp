#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


ros::Publisher cmd_vel_pub;
ros::Publisher takeoff_pub;
std_msgs::Empty takeoff_msg;
geometry_msgs::Twist twist_msg;

double move_forward_time = 10;  // secondsdouble move_forward_time = 5;  // seconds
double turn_time = 1;  // seconds

void move_forward() {
    ROS_INFO("Moving forward...");
    twist_msg.linear.x = 2.5;  // Set linear velocity in x direction
    cmd_vel_pub.publish(twist_msg);
    ros::Duration(move_forward_time).sleep();
}

void stop() {
    ROS_INFO("stop...");
    twist_msg.linear.x = 0.0; 
    twist_msg.angular.z = 0.0; // Set linear velocity in x direction
    cmd_vel_pub.publish(twist_msg);
     ros::Duration(move_forward_time).sleep();
}

void turn() {
    ROS_INFO("Turning...");
    twist_msg.angular.z = 1.5;  // Set angular velocity about z axis
    cmd_vel_pub.publish(twist_msg);
    ros::Duration(turn_time).sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh;

    takeoff_pub = nh.advertise<std_msgs::Empty>("/drone/takeoff", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(10);  // Loop rate of 10 Hz

    // Wait for publisher to be ready
    while (takeoff_pub.getNumSubscribers() == 0) {
        if (!ros::ok()) {
            return 0;
        }
        ROS_INFO_ONCE("Waiting for subscribers on /drone/takeoff...");
        ros::Duration(0.1).sleep();
    }

   
    takeoff_pub.publish(takeoff_msg);
     ROS_INFO("Drone taking off...");

    move_forward();
    stop();
    turn();
    stop();

    ros::spinOnce();  // Process callbacks

    return 0;
}
