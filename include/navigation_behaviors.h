#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include "ba_interfaces.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "helper.h"
#include "json.hpp"
#include "geometry_msgs/msg/point.hpp"

using json = nlohmann::json;
using namespace std::placeholders;

#pragma endregion

#pragma region GoToPose

/**
 * @brief Go to a target location (wraps around `Nav2`)
 *
 */
class GoToPose : public BT::StatefulActionNode
{
public:
  GoToPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor);
  // void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr costmap_publisher_;
  double obstacle_x_{0.0};
  double obstacle_y_{0.0};
  
  std::string action_name_;
  // int count_{0};
  // double nav_goal_tolerance_{0.0};
  // double distance_remaining_ = std::numeric_limits<double>::max();
};

#pragma endregion
