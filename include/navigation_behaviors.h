#pragma region includes

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ba_interfaces.h"
// #include "manipulator.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "helper.h"

#include "json.hpp"

using json = nlohmann::json;

#pragma endregion

#pragma region GoToPose

/**
 * @brief Go to a target location (wraps around `Nav2`)
 *
 */
class GoToPose : public BT::StatefulActionNode
{
public:
  // GoToPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);
  GoToPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  // rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  // Manipulator manipulator_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};    
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string action_name_;
};

#pragma endregion
