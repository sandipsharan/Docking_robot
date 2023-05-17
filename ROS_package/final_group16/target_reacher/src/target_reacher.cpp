/**
 * @file target_reacher.cpp
 * @author Sandeep Thalapanene
 * @author Sandip Sharan Senthil Kumar
 * @author Sourang Sri hari
 * @brief Contains funtions responsible for the movements of the turtlebot
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"

/**
 * @brief Function for rotating the turtlebot
 * 
 * @param msg 
 */
void TargetReacher::velocity_callback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
    if (msg->data)
    {
        // Passing linear and angular velocities
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0.0;
        // Check if the turtlebot reached the final_destination
        if (!target_reached)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing angular velocity");
            vel_msg.angular.z = 0.2;
        }
        else
            vel_msg.angular.z = 0.0;

        // Publishing the velocity command
        velocity_publisher->publish(vel_msg);
        target_reached = true;
    }
}
/**
 * @brief Function for checking the aruco marker and deciding the final_destination
 * 
 * @param marker_msg 
 */
void TargetReacher::aruco_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> marker_msg)
{
    auto marker = marker_msg->marker_ids; // Creating a vector which stores the marker_id values
    if (marker.at(0) == 0)
        final_destination(0);
    else if (marker.at(0) == 1)
        final_destination(1);
    else if (marker.at(0) == 2)
        final_destination(2);
    else if (marker.at(0) == 3)
        final_destination(3);
}

void TargetReacher::listener_callback()
{
    if (a == true)
    {
        geometry_msgs::msg::TransformStamped tf;
        // Look up for the transformation between \robot1\odom and final_destination frames
        // and set goal for turtlebot to reach the second goal
        try
        {
            tf = buffer->lookupTransform("robot1/odom", "final_destination",
                                         tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            return;
        }
        
        // Setting the trasnformed final_destination position as the goal
        m_bot_controller->set_goal(tf.transform.translation.x, tf.transform.translation.y);
    }
}
/**
 * @brief Function for reaching  final position
 * 
 * @param id 
 */
void TargetReacher::final_destination(int id)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = origin_i;
    t.child_frame_id = "final_destination";

/**
 * @brief Transforming and translating the turtlebot final_destination
 * 
 */
    // 
    if (id == 0)
    {
        t.transform.translation.x = x0;
        t.transform.translation.y = y0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
    }
    else if (id == 1)
    {
        t.transform.translation.x = x1;
        t.transform.translation.y = y1;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
    }
    else if (id == 2)
    {
        t.transform.translation.x = x2;
        t.transform.translation.y = y2;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
    }
    else if (id == 3)
    {
        t.transform.translation.x = x3;
        t.transform.translation.y = y3;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
    }
    a = true;
    final_broadcaster->sendTransform(t);
}
