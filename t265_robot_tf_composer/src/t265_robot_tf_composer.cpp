#include "t265_robot_tf_composer/t265_robot_tf_composer.h"

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::TransformStamped transformStamped;

  try {
    tfStamped = tfBuffer.lookupTransform("camera_pose_frame",
                                         "op3/body_link",
                                         ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
    return;
  }

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "op3/body_link";

  // Transform
  transformStamped.transform.translation.x =
    msg->pose.pose.position.x + tfStamped.transform.translation.x;
  transformStamped.transform.translation.y =
    msg->pose.pose.position.y + tfStamped.transform.translation.y;
  transformStamped.transform.translation.z =
    msg->pose.pose.position.z + tfStamped.transform.translation.z;

  // Rotation
  tf2::Quaternion newQ, tfStampedQ, msgQ;
  tf2::convert(tfStamped.transform.rotation, tfStampedQ);
  tf2::convert(msg->pose.pose.orientation, msgQ);

  newQ = tfStampedQ * msgQ;
  newQ.normalize();

  tf2::convert(newQ, transformStamped.transform.rotation);

  br.sendTransform(transformStamped);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t265_robot_tf_composer");

  ros::NodeHandle nh;

  // odom topic subscriber
  ros::Subscriber odom_sub = nh.subscribe("odometry", 10, &odomCallback);

  // tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // ros::Rate rate(125);
  // while (ros::ok())
  //   {
  //     geometry_msgs::TransformStamped tfStamped;
  //     try{
  //       // loop up tf from camera_pose_frame to op3/body_link
  //       tfStamped = tfBuffer.lookupTransform("camera_pose_frame", "op3/body_link",
  //                                            ros::Time(0));
  //     }
  //     catch (tf2::TransformException &ex) {
  //       ROS_WARN("%s",ex.what());
  //       ros::Duration(1.0).sleep();
  //       continue;
  //     }
  //     // calculate odom -> op3/body_link from odom topic and tf and broadcast
  //     ros::spinOnce();
  //     rate.sleep();
  //   }

  ros::spin();

  return 0;
}
