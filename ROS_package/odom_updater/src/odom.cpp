#include <functional>
#include <memory>
#include <sstream>
#include <string>
/**
 * @file odom.cpp
 * @author Sandeep Thalapanane
 * @author Sandip Sharan Senthil Kumar
 * @author Sourang Sri hari
 * @brief This is the tf broadcaster script of the turtlebot
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include "odom_updater/odom.h"
/**
 * @brief This function is responsible for updating the position of the turtlebot in the quartenion format by connecting the parent and the child links
 * 
 * @param msg 
 */
 void FramePublisher::handle_turtle_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "/robot1/odom";                           // Parent_frame
    t.child_frame_id = "/robot1/base_footprint";                  // Child_frame

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;        // Turtlebot's X-coordinate
    t.transform.translation.y = msg->pose.pose.position.y;        // Turtlebot's Y-coordinate
    t.transform.translation.z = 0.0;                              // Turtlebot's Z-coordinate

    //  Quaternion values
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }