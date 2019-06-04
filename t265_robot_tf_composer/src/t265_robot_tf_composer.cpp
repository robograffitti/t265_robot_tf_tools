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

    tfCameraToHeadTilt = tfBuffer.lookupTransform("camera_pose_frame",
                                                  "op3/head_tilt_link",
                                                  ros::Time(0));

    tfHeadTiltToHeadPan = tfBuffer.lookupTransform("op3/head_tilt_link",
                                                   "op3/head_pan_link",
                                                   ros::Time(0));

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
    y_1 = tfCameraToBody.transform.translation.y;
    z_1 = tfCameraToBody.transform.translation.z;
    ROS_INFO("init : %f", x_1);
  } count++;

  tfOdom.header.stamp = ros::Time::now();
  tfOdom.header.frame_id = "odom";
  tfOdom.child_frame_id = "op3/body_link";

  // Transform
  tfOdom.transform.translation.x = msg->pose.pose.position.x
    - (tfCameraToBody.transform.translation.x - x_1);

  tfOdom.transform.translation.y = msg->pose.pose.position.y
    - (tfCameraToBody.transform.translation.y - y_1);
  tfOdom.transform.translation.z = msg->pose.pose.position.z
    - (tfCameraToBody.transform.translation.z - z_1);

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
  x_1 = 0; y_1 = 0; z_1 = 0;
  // x_2 = 0; y_2 =0; z_2 = 0;

  ros::init(argc, argv, "t265_robot_tf_composer");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/camera/odom/sample", 10, &odomCallback);
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::spin();

  return 0;
}
