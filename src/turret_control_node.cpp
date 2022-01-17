#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <thread>
#include <string>
#include <mutex>

#include <tf2/LinearMath/Quaternion.h>
#include <turret_control_node/TurretAngle.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#define DEG_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)


ros::NodeHandle* node;
nav_msgs::Odometry odom;


float get_turret_theta( float x, float y,
                        float robot_speed_x, float robot_speed_y, float dt,
                        float target_x, float target_y )
{
    float robot_pred_x = x + robot_speed_x * dt;
    float robot_pred_y = y + robot_speed_y * dt;
    float theta = atan2 (target_y-robot_pred_y, (target_x-robot_pred_x));


    theta = theta * RAD_TO_DEG;
    return theta;
}

void odom_callback(const nav_msgs::Odometry& msg)
{
	static ros::Publisher turret_angle_pub = node->advertise<geometry_msgs::PoseStamped>("turret_angle", 1);
	geometry_msgs::PoseStamped angle;

 	odom = msg;
 	float theta = get_turret_theta(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.twist.twist.linear.x, odom.twist.twist.linear.y, .1, 8.2296, 4.1148);

    tf2::Quaternion Up;
    Up.setRPY(0,0,theta*DEG_TO_RAD);
    angle.header = odom.header;
    angle.pose = odom.pose.pose;
    angle.pose.orientation.w = Up.getW();
    angle.pose.orientation.x = Up.getX();
    angle.pose.orientation.y = Up.getY();
    angle.pose.orientation.z = Up.getZ();

	turret_angle_pub.publish(angle);

	static ros::Publisher marker_pub = node->advertise<visualization_msgs::Marker>("target", 1);
    visualization_msgs::Marker m;
    m.header = odom.header;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 8.2296;
    m.pose.position.y = 4.1148;
    m.pose.position.z = 0;

    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;

    m.scale.x = 1.3208;
    m.scale.y = 1.3208;
    m.scale.z = 2.6416;

    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 1.0;

    marker_pub.publish(m);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turret_control_node");

	ros::NodeHandle n;

	node = &n;
	ros::Subscriber odom_sub = node->subscribe("/odometry/filtered", 100, odom_callback);

	ros::spin();
	return 0;
}
