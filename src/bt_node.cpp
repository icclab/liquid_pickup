// Main behavior node for Summit-XL

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define LOG_TIME

#ifdef LOG_TIME
#include "time_logger.h"
#endif

#include "manipulator_behaviors.h"  
#include "gripper_behavior.h"
#include "navigation_behaviors.h"
#include "sensors_deploy_behaviors.h"

#include "manipulator.h"
#include "robot.h"

#include <unistd.h>
#include <stdio.h>

#include "ba_interfaces.h"

#include "json.hpp"

using json = nlohmann::json;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node_ = rclcpp::Node::make_shared("bt_node");
  
  // auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(node_);

  std::thread([&executor]() { executor->spin(); }).detach();

  node_->declare_parameter("yaml_file", "arm_positions.yaml");
  node_->declare_parameter("bt_xml", "test_2.xml");

  node_->declare_parameter("coordinates", "[[2.0, 3.0], [4.0, 1.0]]");
  std::string coordinates_array = node_->get_parameter("coordinates").as_string();

  json parsed = json::parse(coordinates_array);

  std::vector<std::vector<double>> coordinates = parsed.get<std::vector<std::vector<double>>>();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<RobotInitializer>("RobotInitializer", node_);
  factory.registerNodeType<ManipulatorGrasp>("Grasp", node_);
  factory.registerNodeType<ManipulatorPregraspPlan>("PregraspPlan", node_);
  factory.registerNodeType<ManipulatorDrop>("Drop", node_);
  factory.registerNodeType<ManipulatorPostgraspRetreat>("RetreatZ", node_);
  factory.registerNodeType<ManipulatorScanPose>("ScanPose", node_);
  factory.registerNodeType<GripperActuator>("ChangeGripper", node_, executor);
  factory.registerNodeType<GoToPose>("GoToPose", node_, executor);
  factory.registerNodeType<SensorsDeploy>("SensorsDeploy", node_, coordinates);
  factory.registerNodeType<ManipulatorPregraspExecute>("PregraspExecute", node_);

  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  std::cerr << xml_models;

  BT::Tree tree;
  try
  {
    std::string bt_xml = node_->get_parameter("bt_xml").as_string(); 
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");
    std::string path_to_xml = package_share_directory + "/config/";

    tree = factory.createTreeFromFile(path_to_xml + bt_xml);
  }
  
  catch (const std::exception &e)
  {
    std::cerr << "Error creating BT from xml file!" << std::endl;
    std::cerr << e.what() << '\n';
  }

  // unsigned server_port = 1667;
  // BT::Groot2Publisher publisher(tree, server_port);
  BT::Groot2Publisher publisher(tree); // default port is 1667

  // Tick the tree until it reaches a terminal state
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  auto start = node_->get_clock()->now().seconds();
      
  #ifdef LOG_TIME
  BATimeLogger::InitFiles();
  #endif

  // status = tree.tickWhileRunning(std::chrono::milliseconds(100)); 
  status = tree.tickWhileRunning(); // default sleep is 10 ms.

  // Output final results
  std::string status_str;
  if (status == BT::NodeStatus::SUCCESS)
  {
    status_str = "SUCCESS";
  }
      
  else
  {
    status_str = "FAILURE";
  }

  auto stop = node_->get_clock()->now().seconds();
  auto seconds = stop - start;

  RCLCPP_INFO(node_->get_logger(), "Done with status %s!", status_str.c_str());
  RCLCPP_INFO(node_->get_logger(), "Used time: %.2lf seconds", seconds);
      
  #ifdef LOG_TIME
    BATimeLogger::CloseFiles();
  #endif

  rclcpp::shutdown();

  return 0;
}
