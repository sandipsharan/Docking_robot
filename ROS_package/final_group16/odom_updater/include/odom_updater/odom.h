#include <functional>
#include <memory>
#include <sstream>
#include <string>
/**
 * @file odom.h
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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>

/*! Class for transform and connect /robot1/odom and /robot1/base_footprint frames*/
class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
      : Node("odom_updater")
  {
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot1/odom", 10,
        std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
  }
/**
 * @brief Call_back function for connecting 
     * /robot1/odom and /robot1/base_footprint frames
 * 
 */
private:
   // Function declaration
    /**
     * @brief Call_back function for connecting 
     * /robot1/odom and /robot1/base_footprint frames
     * 
     * @param msg Pointer to the subcribed odom frame data
     */
  void handle_turtle_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

  // Attributes 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_; // Subscriber for /robot1/odom topic
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;         // Broadcaster for connecting /robot1/odom and /robot1/base_footprint
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}