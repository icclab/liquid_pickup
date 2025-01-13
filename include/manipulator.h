#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#pragma region includes

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Transform.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <stdio.h>
#include "yaml-cpp/yaml.h"
#include <math.h>
#include "helper.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/logger.hpp"
#include <string>

#include "ba_frames_summit_xl.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#pragma endregion

#pragma region defines

#define ROBOT_DESCRIPTION               "robot_description"
#define GROUP_NAME                      "arm"

#define MY_PI                           3.14159
#define PREGRASP_DISTANCE               0.35    // distance offset of ee from  for validation scan
#define TCP_OFFSET_Z                    0.10    // vertical offset
#define TCP_OFFSET_XY                   0.15    // distance offset in XY-plane
#define MANIPULATOR_JOINT_TOLERANCE     0.01
#define MANIPULATOR_TOLERANCE_SMALL     0.01
#define MANIPULATOR_TOLERANCE_PREGRASP  0.015
#define MANIPULATOR_TOLERANCE_MEDIUM    0.05
#define MANIPULATOR_TOLERANCE_LARGE     0.1

#pragma endregion

#pragma region Manipulator

/**
 * @brief Class which actually controls the manipulator
 * 
 */
class Manipulator 
{
public:
    Manipulator(const rclcpp::Node::SharedPtr node);

    moveit_msgs::msg::RobotTrajectory PlanGripperToPose(double target_base_footprint_x_, double target_base_footprint_y_, double target_base_footprint_z_, double target_base_footprint_roll_, double target_base_footprint_pitch_, double target_base_footprint_yaw_);

    moveit::core::MoveItErrorCode ExecuteGripperToPose(moveit_msgs::msg::RobotTrajectory trajectory);

    moveit::core::MoveItErrorCode MoveGripperToPoseLinear(double target_base_footprint_x_, double target_base_footprint_y_, double target_base_footprint_z_, double target_base_footprint_roll_, double target_base_footprint_pitch_, double target_base_footprint_yaw_, double tcp_offset_x, double tcp_offset_y, double tcp_offset_z);

    double MoveLinear(geometry_msgs::msg::Pose end_pose, bool check_collision = true);
    double MoveLinearVec(double x, double y, double z);
    moveit::core::MoveItErrorCode DropObject(void);
    moveit::core::MoveItErrorCode MoveToInitialPosition(void);
    moveit::core::MoveItErrorCode MoveToDrivingPosition(void);
    moveit::core::MoveItErrorCode MoveToScanningPosition(void);

    bool AttachObjectToGripper();
    bool DetachObjectFromGripper();
private:
    // moveit::planning_interface::MoveGroupInterface *manipulator_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> manipulator_;

    geometry_msgs::msg::PoseStamped drop_pose_;
    std::map<std::string, double> initial_position_, driving_position_, scanning_position_;
    void InitializeSummitXlPoses(void);
    void InitializeInitialPose(void);
    void InitializeDrivingPose(void);
    void InitializeScanningPose(void);
    void InitializeDropPose(void);
    rclcpp::Node::SharedPtr node_;
    std::string yaml_file;
    YAML::Node arm_positions;
    std::string attach_detach_object = "liquid_swab";

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr start_state_publisher_;
    // rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory__publisher_;

    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};

#pragma endregion

#endif
