#include "manipulator_behaviors.h"

#pragma region ManipulatorGrasp

/**
 * @brief Construct a new Manipulator Grasp :: Manipulator Grasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

ManipulatorGrasp::ManipulatorGrasp(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name_ = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGrasp::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    BT::Optional<double> base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> base_footprint_yaw = getInput<double>("target_yaw");
    BT::Optional<double> tcp_offset_x = getInput<double>("tcp_offset_x"); 
    BT::Optional<double> tcp_offset_y = getInput<double>("tcp_offset_y"); 
    BT::Optional<double> tcp_offset_z = getInput<double>("tcp_offset_z");

    RCLCPP_INFO(node_->get_logger(), "[%s]: moving gripper to target linearly", action_name_.c_str());

    moveit::core::MoveItErrorCode error_code = manipulator_.MoveGripperToPoseLinear(base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value(), tcp_offset_x.value(), tcp_offset_y.value(), tcp_offset_z.value());
    
    error_message_ = moveit::core::error_code_to_string(error_code);

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorGrasp::onRunning()
{           
    if (error_message_ == "SUCCESS")
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: moved gripper to target linearly", action_name_.c_str());

        return BT::NodeStatus::SUCCESS;
    }
    
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorGrasp::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorGrasp::providedPorts()
{
    return {BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw"), BT::InputPort<double>("tcp_offset_x"), BT::InputPort<double>("tcp_offset_y"), BT::InputPort<double>("tcp_offset_z"), };
}

#pragma endregion

#pragma region ManipulatorPregraspPlan

/**
 * @brief Construct a new Manipulator ManipulatorPregraspPlan:: ManipulatorPregraspPlan Pregrasp object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPregraspPlan::ManipulatorPregraspPlan(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name_ = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    const rclcpp::QoS feedback_sub_qos = rclcpp::QoS(1);

    trajectory_execute_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/summit/trajectory_execute", feedback_sub_qos, std::bind(&ManipulatorPregraspPlan::topic_callback, this, _1));

    wait_set_.add_subscription(trajectory_execute_subscription_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

void ManipulatorPregraspPlan::topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    return;
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspPlan::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    BT::Optional<double> base_footprint_x = getInput<double>("target_x");
    BT::Optional<double> base_footprint_y = getInput<double>("target_y");
    BT::Optional<double> base_footprint_z = getInput<double>("target_z");
    BT::Optional<double> base_footprint_roll = getInput<double>("target_roll");
    BT::Optional<double> base_footprint_pitch = getInput<double>("target_pitch");
    BT::Optional<double> base_footprint_yaw = getInput<double>("target_yaw");
    BT::Optional<double> pregresp_offset = getInput<double>("pregrasp_offset");
    
    setOutput<double>("target_z_cp", base_footprint_z.value());
    setOutput<double>("target_roll_cp", base_footprint_roll.value());
    setOutput<double>("target_pitch_cp", base_footprint_pitch.value());
    setOutput<double>("target_yaw_cp", base_footprint_yaw.value());

    std::string target_frame;

    RCLCPP_INFO(node_->get_logger(), "[%s]: pregrasp started", action_name_.c_str());
    
    bool deploy{false};
    getInput<bool>("deploy", deploy);
    setOutput<bool>("set_deploy", deploy);

    if (deploy)
    {   
        std::vector<std::vector<double>> deploy_coordinates_dynamic_;
        getInput("deploy_coordinates_dynamic", deploy_coordinates_dynamic_);

        double base_x = deploy_coordinates_dynamic_.at(0).at(0);
        double base_y = deploy_coordinates_dynamic_.at(0).at(1);
        
        RCLCPP_INFO(node_->get_logger(), "[%s]: deploying or liquid picking!", action_name_.c_str());
        
        tf2::Transform tf2_transform;
        tf2_transform.setIdentity();

        geometry_msgs::msg::TransformStamped transform;
        try {
        transform = tf_buffer_->lookupTransform(
            BASE_FRAME, MAP_FRAME, tf2::TimePointZero,
            tf2::durationFromSec(5.0));
        
        tf2::fromMsg(transform.transform, tf2_transform);

        tf2::Vector3 point(base_x, base_y, 0);
        point = tf2_transform * point;
        base_x = point.x();
        base_y = point.y();
        } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            node_->get_logger(), "[%s]: Failed to get transformation from %s to %s: %s", action_name_.c_str(),
            MAP_FRAME, BASE_FRAME, ex.what());
        return BT::NodeStatus::FAILURE;
        }

        double angle = atan2(base_y, base_x);
        base_x += pregresp_offset.value() * cos(angle);
        base_y += pregresp_offset.value() * sin(angle); 
            
        plan_trajectory_ = manipulator_.PlanGripperToPose(base_x, base_y, base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value());

        setOutput<double>("target_x_cp", base_x);
        setOutput<double>("target_y_cp", base_y);
    }

    else
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: getting manipulation target coordinates from BT XML input ports!", action_name_.c_str());

        plan_trajectory_ = manipulator_.PlanGripperToPose(base_footprint_x.value(), base_footprint_y.value(), base_footprint_z.value(), base_footprint_roll.value(), base_footprint_pitch.value(), base_footprint_yaw.value());

        setOutput<double>("target_x_cp", base_footprint_x.value());
        setOutput<double>("target_y_cp", base_footprint_y.value());
    }

    setOutput<moveit_msgs::msg::RobotTrajectory>("plan_trajectory", plan_trajectory_);

    RCLCPP_INFO(node_->get_logger(), "[%s]: pregrasp plan finished", action_name_.c_str());
    
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspPlan::onRunning()
{   
    int traj_exec_wait_min{5};
    
    if (!plan_trajectory_.joint_trajectory.header.frame_id.empty())
    {
        if (count_ == 0)
        {
            RCLCPP_WARN(node_->get_logger(), "[%s]: valid trajectory plan received!", action_name_.c_str());
            
            RCLCPP_WARN(node_->get_logger(), "[%s]: check: Execute Trajectory? Waiting for publisher for about %d mins max., Format: $ ros2 topic pub /summit/trajectory_execute std_msgs/msg/Bool \"data: true\" --once", action_name_.c_str(), traj_exec_wait_min);
            count_++;
            return BT::NodeStatus::RUNNING;
        }
        
        int timeout_secs = traj_exec_wait_min * 60;
        
        if (count_ <= timeout_secs)
        {
            // Wait for the subscriber event to trigger. Set a 1 ms margin to trigger a timeout.
            const auto wait_result = wait_set_.wait(std::chrono::milliseconds(1001));
            switch (wait_result.kind()) {
                case rclcpp::WaitResultKind::Ready:
                {
                    std_msgs::msg::Bool take_msg;
                    rclcpp::MessageInfo msg_info;
                    if (trajectory_execute_subscription_->take(take_msg, msg_info)) {
                        bool value_received = take_msg.data;
                        RCLCPP_WARN(node_->get_logger(), "[%s]: check: Received take trajectory execute (0: False, 1: True): %d", action_name_.c_str(), value_received);

                        setOutput<bool>("execute_trajectory", value_received);
                        
                        if (value_received)
                        {
                            RCLCPP_WARN(node_->get_logger(), "[%s]: check: Trajectory execute approved", action_name_.c_str());

                            value_received = false;
                            
                            return BT::NodeStatus::SUCCESS;
                        }

                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(), "[%s]: check: Trajectory execute disapproved", action_name_.c_str());
                            return BT::NodeStatus::FAILURE;
                        }
                    }
                    break;
                }

                case rclcpp::WaitResultKind::Timeout:
                {
                    if (rclcpp::ok()) {
                        RCLCPP_WARN(node_->get_logger(), "[%s]: check: Timeout. No message received yet, still waiting for about %d secs", action_name_.c_str(), timeout_secs - count_);
                    }
                    break;
                }

                default:
                {
                    RCLCPP_ERROR(node_->get_logger(), "[%s]: check: Error. Wait-set failed.", action_name_.c_str());
                }
            }
        }
        
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s]: Timeout! no response received, returning failure", action_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }

        count_++;
    }

    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s]: invalid trajectory plan received, returning failure", action_name_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPregraspPlan::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPregraspPlan::providedPorts()
{
    return {BT::InputPort<double>("target_x"), BT::InputPort<double>("target_y"), BT::InputPort<double>("target_z"), BT::InputPort<double>("pregrasp_offset"), BT::InputPort<double>("target_roll"), BT::InputPort<double>("target_pitch"), BT::InputPort<double>("target_yaw"), BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("plan_trajectory"), BT::OutputPort<bool>("execute_trajectory"), BT::OutputPort<double>("target_x_cp"), BT::OutputPort<double>("target_y_cp"), BT::OutputPort<double>("target_z_cp"), BT::OutputPort<double>("target_roll_cp"), BT::OutputPort<double>("target_pitch_cp"), BT::OutputPort<double>("target_yaw_cp"), BT::InputPort<std::vector<std::vector<double>>>("deploy_coordinates_dynamic"), BT::InputPort<bool>("deploy"), BT::OutputPort<bool>("set_deploy")};
}

#pragma endregion

#pragma region ManipulatorPregraspExecute

/**
 * @brief Construct a new Manipulator Pregrasp Execute:: Manipulator Pregrasp Execute object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPregraspExecute::ManipulatorPregraspExecute(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{   
    action_name_ = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspExecute::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    bool execute_trajectory{false};
    getInput<bool>("execute_trajectory", execute_trajectory);
    
    if (execute_trajectory)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: signaled to execute trajectory, executing!", action_name_.c_str());
        
        BT::Optional<moveit_msgs::msg::RobotTrajectory> trajectory = getInput<moveit_msgs::msg::RobotTrajectory>("plan_trajectory");

        moveit::core::MoveItErrorCode error_code = manipulator_.ExecuteGripperToPose(trajectory.value());

        error_message_ = moveit::core::error_code_to_string(error_code);

        RCLCPP_INFO(node_->get_logger(), "[%s]: Error message: %s", action_name_.c_str(), error_message_.c_str());
        
        execute_trajectory = false;

        return BT::NodeStatus::RUNNING;
    }

    else
    {
        RCLCPP_WARN(node_->get_logger(), "[%s]: signaled to not execute trajectory", action_name_.c_str());
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPregraspExecute::onRunning()
{
    bool set_deploy{false};
    getInput<bool>("set_deploy", set_deploy);

    if (error_message_ == "SUCCESS")
    {
        if (set_deploy)
        {
            std::vector<std::vector<double>> deploy_coordinates_dynamic;
            getInput("deploy_coordinates_dynamic", deploy_coordinates_dynamic);
            deploy_coordinates_dynamic.erase(deploy_coordinates_dynamic.begin());
            setOutput<std::vector<std::vector<double>>>("deploy_coordinates_dynamic", deploy_coordinates_dynamic);

            RCLCPP_INFO(node_->get_logger(), "[%s]: %lu sensor(s) yet to be deployed", action_name_.c_str(), deploy_coordinates_dynamic.size());
        }
        
        return BT::NodeStatus::SUCCESS;
    }
    
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPregraspExecute::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPregraspExecute::providedPorts()
{
    return {BT::InputPort<moveit_msgs::msg::RobotTrajectory>("plan_trajectory"), BT::InputPort<bool>("execute_trajectory"), BT::BidirectionalPort<std::vector<std::vector<double>>>("deploy_coordinates_dynamic"), BT::InputPort<bool>("set_deploy")};
}

#pragma endregion

#pragma region ManipulatorPostgraspRetreat

/**
 * @brief Construct a new Manipulator Postgrasp Retreat:: Manipulator Postgrasp Retreat object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */
ManipulatorPostgraspRetreat::ManipulatorPostgraspRetreat(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name_ = this->name();

    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    RCLCPP_INFO(node_->get_logger(), "[%s]: post grasp started", action_name_.c_str());
    
    BT::Optional<double> add_pos_z = getInput<double>("add_pos_z");
    
    double res = manipulator_.MoveLinearVec(0, 0, add_pos_z.value());
    
    RCLCPP_INFO(node_->get_logger(), "[%s]: post grasp finished with result: %f", action_name_.c_str(), res);
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorPostgraspRetreat::onRunning()
{   
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorPostgraspRetreat::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorPostgraspRetreat::providedPorts()
{
    return {BT::InputPort<double>("add_pos_z")};
}

#pragma endregion

#pragma region ManipulatorDrop

/**
 * @brief Construct a new Manipulator Drop :: Manipulator Drop an object
 * 
 * @param name The name of the behavior
 */
ManipulatorDrop::ManipulatorDrop(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name_ = this->name();

    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDrop::onStart()
{
    // LOG_MANI_START(action_name_);

    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    RCLCPP_INFO(node_->get_logger(), "[%s]: going to drop object", action_name_.c_str());
    manipulator_.DropObject();
    RCLCPP_INFO(node_->get_logger(), "[%s]: object dropped", action_name_.c_str());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorDrop::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorDrop::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorDrop::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorDrop"), "returning empty BT::PortsList!");
    return {};
}


#pragma endregion

#pragma region ManipulatorScanPose

/**
 * @brief Construct a new Manipulator Scan Pose:: Manipulator Scan Pose object
 * 
 * @param name The name of the behavior
 */
ManipulatorScanPose::ManipulatorScanPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name_ = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[%s]: moving EE to scan position", action_name_.c_str());
    manipulator_.MoveToScanningPosition();
    RCLCPP_INFO(node_->get_logger(), "[%s]: moved EE to scan position", action_name_.c_str());
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 * 
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus ManipulatorScanPose::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 * 
 */
void ManipulatorScanPose::onHalted() {}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList ManipulatorScanPose::providedPorts()
{
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorScanPose"), "returning empty BT::PortsList!");
    return {};
}

#pragma endregion