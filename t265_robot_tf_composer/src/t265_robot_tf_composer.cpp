#include "t265_robot_tf_composer/t265_robot_tf_composer.h"

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tfCameraToBody;
  geometry_msgs::TransformStamped tfCameraToHeadTilt;
  geometry_msgs::TransformStamped tfHeadTiltToHeadPan;
  geometry_msgs::TransformStamped tfHeadPanToBody;
  geometry_msgs::TransformStamped tfOdom;

  try {
    tfCameraToBody = tfBuffer.lookupTransform("op3/body_link",
                                              "camera_pose_frame",
                                              ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
    return;
  }

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

  if (count == 0) {
    x_1 = tfCameraToBody.transform.translation.x;
    // x_2 = tfHeadPanToBody.transform.translation.x;
    y_1 = tfCameraToBody.transform.translation.y;
    // y_2 = tfHeadTiltToHeadPan.transform.translation.y;
    z_1 = tfCameraToBody.transform.translation.z;
    // z_2 = tfHeadTiltToHeadPan.transform.translation.z;
    ROS_INFO("init : %f", x_1);
  } count++;

  tfOdom.header.stamp = ros::Time::now();
  tfOdom.header.frame_id = "odom";
  tfOdom.child_frame_id = "op3/body_link";

  // Transform
  tfOdom.transform.translation.x = msg->pose.pose.position.x
    // + tfCameraToBody.transform.translation.x
    // + tfHeadPanToBody.transform.translation.x
    - (tfCameraToBody.transform.translation.x - x_1)
    // - (tfHeadTiltToHeadPan.transform.translation.x - x_2)
    // - x_1 - x_2;
    ; // + or - tf caused by rotation

  ROS_INFO("odom x : %f", msg->pose.pose.position.x);
  ROS_INFO("tf from body to cam : %f", tfCameraToBody.transform.translation.x);
  ROS_INFO("init : %f", x_1);

  tfOdom.transform.translation.y = msg->pose.pose.position.y
    // - tfCameraToBody.transform.translation.y
    // + tfHeadPanToBody.transform.translation.y
    - (tfCameraToBody.transform.translation.y - y_1)
    // - (tfHeadTiltToHeadPan.transform.translation.y - y_2)
    // - y_1 - y_2;
    ; // same as x
  tfOdom.transform.translation.z = msg->pose.pose.position.z
    // - tfCameraToBody.transform.translation.z
    // + tfHeadPanToBody.transform.translation.z
    - (tfCameraToBody.transform.translation.z - z_1)
    // - (tfHeadTiltToHeadPan.transform.translation.z - z_2)
    // - z_1 - z_2;
    ; // same as x, y

  // Rotation
  // tfCameraToHeadTilt tfHeadTiltToHeadPan tfHeadPanToBody tfOdom
  tf2::Quaternion odomQ, headPanBodyQ, headTiltHeadPanQ, camHeadTiltQ, msgQ;
  tf2::convert(tfHeadPanToBody.transform.rotation, headPanBodyQ);
  tf2::convert(tfHeadTiltToHeadPan.transform.rotation, headTiltHeadPanQ);
  tf2::convert(tfCameraToHeadTilt.transform.rotation, camHeadTiltQ);
  tf2::convert(msg->pose.pose.orientation, msgQ);

  odomQ = camHeadTiltQ * headPanBodyQ * headTiltHeadPanQ * msgQ;
  odomQ.normalize();

  tf2::convert(odomQ, tfOdom.transform.rotation);

  br.sendTransform(tfOdom);

}

int main(int argc, char **argv)
{
  x_1 = 0; x_2 = 0; y_1 = 0; y_2 = 0; z_1 =0; z_2 = 0;

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
