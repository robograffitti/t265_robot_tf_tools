#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

double x_1, x_2, y_1, y_2, z_1, z_2;
int count = 0;

tf2_ros::Buffer tfBuffer;
void odomCallback(const nav_msgs::OdometryConstPtr& msg);
