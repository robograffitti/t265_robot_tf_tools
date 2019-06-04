#include "t265_robot_tf_composer/t265_robot_tf_composer.h"

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tfCameraToHeadTilt;
  geometry_msgs::TransformStamped tfHeadTiltToHeadPan;
  geometry_msgs::TransformStamped tfHeadPanToBody;
  geometry_msgs::TransformStamped tfOdom;

  try {
    tfCameraToHeadTilt = tfBuffer.lookupTransform("camera_pose_frame",
                                         "op3/head_tilt_link",
                                         ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
    return;
  }

  try {
    tfHeadTiltToHeadPan = tfBuffer.lookupTransform("op3/head_tilt_link",
                                         "op3/head_pan_link",
                                         ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
    return;
  }

  try {
    tfHeadPanToBody = tfBuffer.lookupTransform("op3/head_pan_link",
                                         "op3/body_link",
                                         ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
    return;
  }

  tfOdom.header.stamp = ros::Time::now();
  tfOdom.header.frame_id = "odom";
  tfOdom.child_frame_id = "op3/body_link";

  // Transform
  tfOdom.transform.translation.x = msg->pose.pose.position.x;
  tfOdom.transform.translation.y = msg->pose.pose.position.y;
  tfOdom.transform.translation.z = msg->pose.pose.position.z;

  // Rotation
  // tfCameraToHeadTilt tfHeadTiltToHeadPan tfHeadPanToBody tfOdom
  tf2::Quaternion odomQ, headPanBodyQ, headTiltHeadPanQ, camHeadTiltQ, msgQ;
  tf2::convert(tfHeadPanToBody.transform.rotation, headPanBodyQ);
  tf2::convert(tfHeadTiltToHeadPan.transform.rotation, headTiltHeadPanQ);
  tf2::convert(tfCameraToHeadTilt.transform.rotation, camHeadTiltQ);
  tf2::convert(msg->pose.pose.orientation, msgQ);

  odomQ = headPanBodyQ * headTiltHeadPanQ  * camHeadTiltQ * msgQ;
  odomQ.normalize();

  tf2::convert(odomQ, tfOdom.transform.rotation);

  br.sendTransform(tfOdom);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t265_robot_tf_composer");

  ros::NodeHandle nh;

  // odom topic subscriber
  ros::Subscriber odom_sub = nh.subscribe("/camera/odom/sample", 10, &odomCallback);

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
