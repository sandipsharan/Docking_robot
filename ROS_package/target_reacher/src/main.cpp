
#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include "bot_controller/bot_controller.h"
/**
 * @file main.cpp
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
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto bot_controller = std::make_shared<BotController>("bot_controller_robot", "robot1");
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<TargetReacher>(bot_controller);
    exec.add_node(node);
    exec.add_node(bot_controller);
    exec.spin();
    rclcpp::shutdown();
}