#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

/**
 * @file target_reacher.h
 * @author Sandeep Thalapanane
 * @author Sandip Sharan Senthil Kumar
 * @author Sourang Sri hari
 * @brief Conatins the functions responsible for communication of the messages between the nodes
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

using namespace std::chrono_literals;

/*! Class for reaching first_destination and final_destination */
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;

        // Declaring paramters from final_params.yaml
        x0 = this->declare_parameter<double>("final_destination.aruco_0.x");
        y0 = this->declare_parameter<double>("final_destination.aruco_0.y");
        x1 = this->declare_parameter<double>("final_destination.aruco_1.x");
        y1 = this->declare_parameter<double>("final_destination.aruco_1.y");
        x2 = this->declare_parameter<double>("final_destination.aruco_2.x");
        y2 = this->declare_parameter<double>("final_destination.aruco_2.y");
        x3 = this->declare_parameter<double>("final_destination.aruco_3.x");
        y3 = this->declare_parameter<double>("final_destination.aruco_3.y");

        auto a = this->declare_parameter<double>("aruco_target.x");
        auto b = this->declare_parameter<double>("aruco_target.y");

        // Setting the goal co-ordinates
        m_bot_controller->set_goal(a, b);
        origin_i = this->declare_parameter<std::string>("final_destination.frame_id");

        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
        subscriber_goal = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10, std::bind(&TargetReacher::velocity_callback, this, std::placeholders::_1));
        subscriber_aruco_marker = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));
        final_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
        timer = this->create_wall_timer(1s, std::bind(&TargetReacher::listener_callback, this));
    }

private:
    // Attributes
    bool a;
    double x0;
    double y0;
    double x1;
    double y1;
    double x2;
    double y2;
    double x3;
    double y3;
    bool target_reached = false;
    std::string origin_i;
    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_goal;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscriber_aruco_marker;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> final_broadcaster;
    rclcpp::TimerBase::SharedPtr timer{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> buffer;
     


    /**
     * @brief Function for reaching the final destination.
     *
     * @param id Aruco_marker ID.
     * @param a Assigned Bool value.
     */
    void final_destination(int id);

    /**
     * @brief Function for reaching the final destination.
     *
     * @param msg Pointer for pointing to the current data value.
     * @param target_reached Bool value for stopping the rotation after reaching the final destination.
     */
    void velocity_callback(const std::shared_ptr<std_msgs::msg::Bool> msg);

    /**
     * @brief Function for reaching the final destination.
     *
     * @param marker_msg Pointer which points to marker_IDs
     * @param marker Vector which stores the value of the marker_IDs.
     */
    void aruco_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg);

    /**
     * @brief Function for reaching the final destination.
     *
     * @param a Bool value.
     */
    void listener_callback();
};
