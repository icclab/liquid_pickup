#include "manipulator.h"

#pragma region Manipulator

#pragma region public

/**
 * @brief Construct a new Manipulator:: Manipulator object
 * 
 */
Manipulator::Manipulator(const rclcpp::Node::SharedPtr node)
{
    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Manipulator"), "nullptr was passed!");
        return;
    }
       
    node_ = node;

    RCLCPP_INFO(node_->get_logger(), "Node shared pointer was passed!");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    yaml_file = node_->get_parameter("yaml_file").as_string();

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("liquid_pickup");

    std::string path_to_yaml = package_share_directory + "/config/";

    arm_positions = YAML::LoadFile(path_to_yaml + yaml_file);

    Manipulator::InitializeSummitXlPoses();

    moveit::planning_interface::MoveGroupInterface::Options manipulator_options_(GROUP_NAME, ROBOT_DESCRIPTION, "/summit");

    manipulator_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, manipulator_options_);
    
    manipulator_->allowReplanning(true);

    // node_->get_parameter("moveit_velocity_scale", v_scale_);
    // node_->get_parameter("moveit_acceleration_scale", a_scale_);

    // manipulator_->setMaxVelocityScalingFactor(v_scale_);
    // manipulator_->setMaxAccelerationScalingFactor(a_scale_);

    // RCLCPP_WARN(node_->get_logger(), "manipulator: moveit: velocity scale = %f", v_scale_);
    // RCLCPP_WARN(node_->get_logger(), "manipulator: moveit: acceleration scale = %f", a_scale_);

    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, BASE_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC, manipulator_->getRobotModel());
}

/**
 * @brief Moves the gripper to a pose in a certain distance off the target
 * 
 * @param target_pose The endpose to reach
 * @param offset The offset to the end pose
 * @return moveit_msgs::msg::RobotTrajectory The robot trajectory
 */
moveit_msgs::msg::RobotTrajectory Manipulator::PlanGripperToPose(double target_base_footprint_x_, double target_base_footprint_y_, double target_base_footprint_z_, double target_base_footprint_roll_, double target_base_footprint_pitch_, double target_base_footprint_yaw_)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_PREGRASP);

    geometry_msgs::msg::PoseStamped target_base_footprint;

    target_base_footprint.header.stamp = node_->get_clock()->now();
    target_base_footprint.header.frame_id = BASE_FRAME;
    target_base_footprint.pose.position.x = target_base_footprint_x_;
    target_base_footprint.pose.position.y = target_base_footprint_y_;
    target_base_footprint.pose.position.z = target_base_footprint_z_;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(target_base_footprint_roll_, target_base_footprint_pitch_, target_base_footprint_yaw_);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    target_base_footprint.pose.orientation = msg_quat;

    RCLCPP_INFO(node_->get_logger(), "MoveIt going to: header.frame_id: %s, x: %f, y: %f, z: %f, rotation qx: %f, qy: %f, qz: %f, qw: %f", target_base_footprint.header.frame_id.c_str(), target_base_footprint.pose.position.x, target_base_footprint.pose.position.y, target_base_footprint.pose.position.z, target_base_footprint.pose.orientation.x, target_base_footprint.pose.orientation.y, target_base_footprint.pose.orientation.z, target_base_footprint.pose.orientation.w);

    manipulator_->setPoseTarget(target_base_footprint);
    
    manipulator_->setPlanningTime(10.0);

    RCLCPP_INFO(node_->get_logger(), "Reference frame: %s", manipulator_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", manipulator_->getEndEffectorLink().c_str());
    
    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (manipulator_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "planning succesful! visualizing plan as trajectory line");
        
        // const moveit::core::JointModelGroup* joint_model_group = manipulator_->getCurrentState()->getJointModelGroup(GROUP_NAME);
        // visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        
        Manipulator::VisualizeArmTrajectory(my_plan);

        return my_plan.trajectory_;
    }

    else
    {
        RCLCPP_INFO(node_->get_logger(), "planning failed, returning empty trajectory");
        moveit_msgs::msg::RobotTrajectory empty_trajectory;
        return empty_trajectory;
    }
}

/**
 * @brief Moves the gripper to a joint goal
 * 
 * @param joint_goal The joint_goal to reach
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::MoveGripperToJoint(std::string joint_goal, double moveit_velocity_scale, double moveit_acceleration_scale)
{
    std::vector<double> joint_angles = arm_positions[joint_goal].as<std::vector<double>>();

    joint_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    joint_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    joint_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    joint_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    joint_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    joint_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);

    moveit::core::MoveItErrorCode code;

    manipulator_->setMaxVelocityScalingFactor(moveit_velocity_scale);
    manipulator_->setMaxAccelerationScalingFactor(moveit_acceleration_scale);

    RCLCPP_WARN(node_->get_logger(), "manipulator: UPDATE moveit: velocity scale = %f", moveit_velocity_scale);
    RCLCPP_WARN(node_->get_logger(), "manipulator: UPDATE moveit: acceleration scale = %f", moveit_acceleration_scale);
    
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(joint_position_);
    manipulator_->setPlanningTime(30);
    code = Manipulator::PlanAndExecute();

    // if (joint_goal == "pick_swab")
    // {
    //     code = Manipulator::MoveToPickSwabPosition();
    // }

    // else if (joint_goal == "deploy")
    // {
    //     code = Manipulator::MoveToDeployPosition();
    // }

    // else
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "unknown joint goal given, returning failure");
    //     code = moveit::core::MoveItErrorCode::FAILURE;
    // }
    
    return code;
}

void Manipulator::VisualizeArmTrajectory(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{
    const moveit::core::JointModelGroup* joint_model_group = manipulator_->getCurrentState()->getJointModelGroup(GROUP_NAME);
    visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group); 
}

moveit::core::MoveItErrorCode Manipulator::ExecuteGripperToPose(moveit_msgs::msg::RobotTrajectory trajectory)
{
    // Execute the plan
    if (!trajectory.joint_trajectory.header.frame_id.empty())
    {
        RCLCPP_INFO(node_->get_logger(), "valid trajectory received! executing");
        return manipulator_->execute(trajectory);
    }
    
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "empty trajectory received, returning failure");
        return moveit::core::MoveItErrorCode::FAILURE;
    }
}

/**
 * @brief Moves the gripper to the target linearly
 * 
 * @param target_pose The endpose to reach
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::MoveGripperToPoseLinear(double target_base_footprint_x_, double target_base_footprint_y_, double target_base_footprint_z_, double target_base_footprint_roll_, double target_base_footprint_pitch_, double target_base_footprint_yaw_, double tcp_offset_x, double tcp_offset_y, double tcp_offset_z)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_SMALL);

    geometry_msgs::msg::PoseStamped target_base_footprint;

    target_base_footprint.header.stamp = node_->get_clock()->now();
    target_base_footprint.header.frame_id = BASE_FRAME; 
    target_base_footprint.pose.position.x = target_base_footprint_x_;
    target_base_footprint.pose.position.y = target_base_footprint_y_;
    target_base_footprint.pose.position.z = target_base_footprint_z_;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(target_base_footprint_roll_, target_base_footprint_pitch_, target_base_footprint_yaw_);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    target_base_footprint.pose.orientation = msg_quat;

    target_base_footprint.pose.position.x += tcp_offset_x;
    target_base_footprint.pose.position.y += tcp_offset_y;
    target_base_footprint.pose.position.z += tcp_offset_z;
    
    RCLCPP_INFO(node_->get_logger(), "MoveIt: linearly going to: header.frame_id: %s, x: %f, y: %f, z: %f, rotation qx: %f, qy: %f, qz: %f, qw: %f", target_base_footprint.header.frame_id.c_str(), target_base_footprint.pose.position.x, target_base_footprint.pose.position.y, target_base_footprint.pose.position.z, target_base_footprint.pose.orientation.x, target_base_footprint.pose.orientation.y, target_base_footprint.pose.orientation.z, target_base_footprint.pose.orientation.w);

    double res =  MoveLinear(target_base_footprint.pose, false);

    RCLCPP_INFO(node_->get_logger(), "result: %f", res);

    return moveit::core::MoveItErrorCode::SUCCESS;
}

/**
 * @brief Moves the gripper linearly
 * 
 * @param end_pose The pose to reach
 * @param check_collision Bool whether to check for collisions or not
 * @return double Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints. Return -1.0 in case of error.
 */
double Manipulator::MoveLinear(geometry_msgs::msg::Pose end_pose, bool check_collision)
{
    std::vector<geometry_msgs::msg::Pose> direction;
    direction.push_back(end_pose);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double res = manipulator_->computeCartesianPath(direction, 0.01, 10, trajectory, check_collision);
    if (res >= 0)
    {
        manipulator_->execute(trajectory);
    }
    return res;
}

/**
 * @brief Moves the gripper linearly based on three coordinates
 * 
 * @param x The x coordinate
 * @param y The y coordinate
 * @param z The z coordinate
 * @return double Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints. Return -1.0 in case of error.
 */
double Manipulator::MoveLinearVec(double x, double y, double z){
    // geometry_msgs::msg::PoseStamped ee = manipulator_->getPoseTarget();
    
    // workaround for above function
    geometry_msgs::msg::PoseStamped ee;
    ee.header.frame_id = "arm_tool0";
    
    geometry_msgs::msg::TransformStamped transform_ee_base_frame;

    try {
        transform_ee_base_frame = tf_buffer_->lookupTransform(
            BASE_FRAME, ee.header.frame_id,
            tf2::TimePointZero);
    }   catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(),
            "Could not transform %s to %s: %s",
            BASE_FRAME, ee.header.frame_id.c_str(), ex.what());
    }

    geometry_msgs::msg::PoseStamped ee_base_frame;

    ee_base_frame.header = transform_ee_base_frame.header;
    ee_base_frame.pose.position.x = transform_ee_base_frame.transform.translation.x;
    ee_base_frame.pose.position.y = transform_ee_base_frame.transform.translation.y;
    ee_base_frame.pose.position.z = transform_ee_base_frame.transform.translation.z;
    ee_base_frame.pose.orientation.x = transform_ee_base_frame.transform.rotation.x;
    ee_base_frame.pose.orientation.y = transform_ee_base_frame.transform.rotation.y;
    ee_base_frame.pose.orientation.z = transform_ee_base_frame.transform.rotation.z;
    ee_base_frame.pose.orientation.w = transform_ee_base_frame.transform.rotation.w;

    ee_base_frame.pose.position.x += x;
    ee_base_frame.pose.position.y += y;
    ee_base_frame.pose.position.z += z;
    
    RCLCPP_INFO(node_->get_logger(), "retreating vertically in x axis by %f meters", x);
    RCLCPP_INFO(node_->get_logger(), "retreating vertically in y axis by %f meters", y);
    RCLCPP_INFO(node_->get_logger(), "retreating vertically in z axis by %f meters", z);

    double res = MoveLinear(ee_base_frame.pose, false);
    return res;
}

/**
 * @brief Moves the UR5-Robot to its drop position
 * 
 * @return moveit::core::MoveItErrorCode The errorcode
 */
moveit::core::MoveItErrorCode Manipulator::DropObject(void)
{
    manipulator_->setGoalPositionTolerance(MANIPULATOR_TOLERANCE_LARGE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setPoseTarget(drop_pose_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

moveit::core::MoveItErrorCode Manipulator::PlanAndExecute()
{
    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (manipulator_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "planning succesful! visualizing plan");
        
        Manipulator::VisualizeArmTrajectory(my_plan);

        moveit::core::MoveItErrorCode error_code = manipulator_->execute(my_plan);
        std::string error_message = moveit::core::error_code_to_string(error_code);

        if (error_message == "SUCCESS")
        {
            RCLCPP_INFO(node_->get_logger(), "executed plan!");
            return error_code;
        }

        else
        {
            RCLCPP_ERROR(node_->get_logger(), "execution failed!, returning failure");
            return error_code;
        }
    }

    else
    {
        RCLCPP_INFO(node_->get_logger(), "planning failed, returning failure");
        return moveit::core::MoveItErrorCode::FAILURE;
    }
}

/**
 * @brief Moves the UR5-Robot to its initial position due to the swab container.
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToInitialPosition(void) 
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(initial_position_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

/**
 * @brief Moves the UR5-Robot to pick swab position.
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToPickSwabPosition(void) 
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(pick_swab_);
    manipulator_->setPlanningTime(30);
    // return manipulator_->move();
    return Manipulator::PlanAndExecute();
}

/**
 * @brief Moves the UR5-Robot to deploy position.
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToDeployPosition(void) 
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(deploy_);
    manipulator_->setPlanningTime(30);
    // return manipulator_->move();
    return Manipulator::PlanAndExecute();
}

/**
 * @brief Moves the UR5-Robot to its driving position.
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToDrivingPosition(void) 
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(driving_position_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

/**
 * @brief Moves the UR5-Robot to its scanning position
 * 
 * @return moveit::core::MoveItErrorCode 
 */
moveit::core::MoveItErrorCode Manipulator::MoveToScanningPosition(void)
{
    manipulator_->setGoalJointTolerance(MANIPULATOR_JOINT_TOLERANCE);
    manipulator_->setPoseReferenceFrame(BASE_FRAME);
    manipulator_->setJointValueTarget(scanning_position_);
    manipulator_->setPlanningTime(30);
    return manipulator_->move();
}

bool Manipulator::AttachObjectToGripper()
{
    // We will use the
    // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // Attaching objects to the robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // You can attach an object to the robot, so that it moves with the robot geometry.
    // This simulates picking up the object for the purpose of manipulating it.
    // The motion planning should avoid collisions between objects as well.
    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = attach_detach_object;

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.1;
    cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.0135;

    // We define the frame/pose for this cylinder so that it appears in the gripper.
    object_to_attach.header.frame_id = manipulator_->getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.2;

    // First, we add the object to the world (without using a vector).
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    RCLCPP_INFO(node_->get_logger(), "Attach the object to the robot");
    std::vector<std::string> touch_links;
    touch_links.push_back("left_inner_finger");
    touch_links.push_back("right_inner_finger");
    touch_links.push_back("left_inner_finger_pad");
    touch_links.push_back("right_inner_finger_pad");
    bool result = manipulator_->attachObject(object_to_attach.id, "arm_tool0", touch_links);
    return result;
}

bool Manipulator::DetachObjectFromGripper()
{
    // Detaching and Removing Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Now, let's detach the cylinder from the robot's gripper.
    RCLCPP_INFO(node_->get_logger(), "Detach the object from the robot");

    moveit_msgs::msg::CollisionObject object_to_detach;
    object_to_detach.id = attach_detach_object;

    bool result = manipulator_->detachObject(object_to_detach.id);
    return result;
}

#pragma endregion

#pragma region private

/**
 * @brief Initializes several given poses
 * 
 */
void Manipulator::InitializeSummitXlPoses()
{
    Manipulator::InitializeInitialPose();
    Manipulator::PickSwabPose();
    Manipulator::DeployPose();
    Manipulator::InitializeDrivingPose();
    Manipulator::InitializeScanningPose();
    Manipulator::InitializeDropPose();
}

/**
 * @brief Initializes the initial pose due to a bug (-J argument not working in launch file).
 *        see also https://answers.ros.org/question/242151/how-to-set-initial-pose-to-ur5-in-gazebo/ 
 *        no workaround worked reliable for that case 
 * 
 */
void Manipulator::InitializeInitialPose()
{
    std::vector<double> joint_angles = arm_positions["initial_joint_angles"].as<std::vector<double>>();

    initial_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    initial_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    initial_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    initial_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    initial_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    initial_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief Initializes the pick swab pose
 * 
 */
void Manipulator::PickSwabPose()
{
    std::vector<double> joint_angles = arm_positions["pick_swab"].as<std::vector<double>>();

    pick_swab_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    pick_swab_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    pick_swab_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    pick_swab_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    pick_swab_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    pick_swab_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief Initializes the pick swab pose
 * 
 */
void Manipulator::DeployPose()
{
    std::vector<double> joint_angles = arm_positions["deploy"].as<std::vector<double>>();

    deploy_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    deploy_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    deploy_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    deploy_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    deploy_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    deploy_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief Initializes the driving pose
 * 
 */
void Manipulator::InitializeDrivingPose()
{
    std::vector<double> joint_angles = arm_positions["driving_joint_angles"].as<std::vector<double>>();

    driving_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    driving_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    driving_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    driving_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    driving_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    driving_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief Initializes the scanning pose
 * 
 */
void Manipulator::InitializeScanningPose()
{
    std::vector<double> joint_angles = arm_positions["scan_position_joint_angles"].as<std::vector<double>>();

    scanning_position_["arm_shoulder_pan_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[0]);
    scanning_position_["arm_shoulder_lift_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[1]);
    scanning_position_["arm_elbow_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[2]);
    scanning_position_["arm_wrist_1_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[3]);
    scanning_position_["arm_wrist_2_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[4]);
    scanning_position_["arm_wrist_3_joint"]=ba_helper::ConvertDegreesToRadians(joint_angles[5]);
}

/**
 * @brief initialize drop zone: above swab container facing downwards
 *
 */
void Manipulator::InitializeDropPose()
{
    std::vector<double> pose = arm_positions["dropping_position"].as<std::vector<double>>();

    drop_pose_.header.frame_id = BASE_FRAME;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(ba_helper::ConvertDegreesToRadians(pose[3]), ba_helper::ConvertDegreesToRadians(pose[4]), ba_helper::ConvertDegreesToRadians(pose[5]));
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

    drop_pose_.pose.orientation = msg_quat;
    drop_pose_.pose.position.x = pose[0];
    drop_pose_.pose.position.y = pose[1];
    drop_pose_.pose.position.z = pose[2];
}

#pragma endregion
