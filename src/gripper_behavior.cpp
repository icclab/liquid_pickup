#include "gripper_behavior.h"

#pragma region public

/**
 * @brief Construct a new Gripper Actuator:: Gripper Actuator object
 * 
 * @param name The name of the behavior
 * @param config The node configuration
 */

// GripperActuator::GripperActuator(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::SingleThreadedExecutor::SharedPtr executor): BT::SyncActionNode(name, config)
GripperActuator::GripperActuator(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor): BT::StatefulActionNode(name, config), manipulator_(node)
{
    action_name_ = this->name();
    
    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }

    if (executor != nullptr)
    {
        executor_ = executor;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Executor shared pointer was passed!", action_name_.c_str());
    }

    action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_, "/summit/robotiq_gripper_controller/gripper_cmd");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Action server not available after waiting", action_name_.c_str());
    return;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GripperActuator::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());
    
    BT::Optional<double> gripper_position = getInput<double>("position");
    BT::Optional<double> gripper_max_effort = getInput<double>("max_effort");
    RCLCPP_INFO(node_->get_logger(), "[%s]: Gripping -> value: %f", action_name_.c_str(), gripper_position.value());

    auto grippercommand_msg = control_msgs::action::GripperCommand::Goal(); 
    grippercommand_msg.command.position = gripper_position.value();
    grippercommand_msg.command.max_effort = gripper_max_effort.value();

    RCLCPP_INFO(node_->get_logger(), "[%s]: Sending goal", action_name_.c_str());

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client_->async_send_goal(grippercommand_msg);

    // if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    // rclcpp::FutureReturnCode::SUCCESS)
    // {
    // RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    // return BT::NodeStatus::FAILURE;
    // }

    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Goal was rejected by server", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
};

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GripperActuator::onRunning()
{
    // Wait for the server to be done with the goal
    auto result_future = action_client_->async_get_result(goal_handle_);

    RCLCPP_INFO(node_->get_logger(), "[%s]: Waiting for result", action_name_.c_str());

    rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            // RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            RCLCPP_WARN(node_->get_logger(), "[%s]: Goal was aborted, but still continuing!", action_name_.c_str());
            // return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            // RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            RCLCPP_WARN(node_->get_logger(), "[%s]: Goal was canceled, but still continuing!", action_name_.c_str());
            // return BT::NodeStatus::FAILURE;
        default:
            // RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            RCLCPP_WARN(node_->get_logger(), "[%s]: Unknown result code, but still continuing!", action_name_.c_str());
            // return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: result received", action_name_.c_str());

    double position = wrapped_result.result->position; // The current gripper gap size (in meters)
    RCLCPP_INFO(node_->get_logger(), "[%s]: current gripper gap size (in meters) = %f", action_name_.c_str(), position);
    double effort = wrapped_result.result->effort;    // The current effort exerted (in Newtons)
    RCLCPP_INFO(node_->get_logger(), "[%s]: current effort exerted (in Newtons) = %f", action_name_.c_str(), effort);
    bool stalled = wrapped_result.result->stalled;      // True iff the gripper is exerting max effort and not moving
    RCLCPP_INFO(node_->get_logger(), "[%s]: stalled (0: False, 1: True)? = %d", action_name_.c_str(), stalled);
    bool reached_goal = wrapped_result.result->reached_goal; // True iff the gripper position has reached the commanded setpoint

    double gripper_cmd_postion{0.0};
    getInput<double>("position", gripper_cmd_postion);
    RCLCPP_INFO(node_->get_logger(), "[%s]: commanded gripper gap size (in meters) = %f", action_name_.c_str(), gripper_cmd_postion);

    double gripper_goal_tolerance{0.0}; 
    getInput<double>("goal_tolerance", gripper_goal_tolerance);
    RCLCPP_INFO(node_->get_logger(), "[%s]: maximum unsigned goal tolerance (in meters) = %f", action_name_.c_str(), gripper_goal_tolerance);

    double goal_diff{0.0};
    goal_diff = position - gripper_cmd_postion;
    RCLCPP_INFO(node_->get_logger(), "[%s]: gripper goal difference (in meters) = %f", action_name_.c_str(), goal_diff);

    if (std::fabs(goal_diff) <= gripper_goal_tolerance)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s]: goal reached", action_name_.c_str());

        bool attach_detach{false};

        std::string attachordetach;
        getInput("attach_or_detach", attachordetach);
        
        if (attachordetach == "attach")
        {
            attach_detach = manipulator_.AttachObjectToGripper();
            RCLCPP_INFO(node_->get_logger(), "[%s]: attached (0: False, 1: True)? = %d", action_name_.c_str(), attach_detach);
        }

        else if (attachordetach == "detach")
        {
            attach_detach = manipulator_.DetachObjectFromGripper();
            RCLCPP_INFO(node_->get_logger(), "[%s]: detached (0: False, 1: True)? = %d", action_name_.c_str(), attach_detach);
        }
        
        else
        {
            RCLCPP_WARN(node_->get_logger(), "[%s]: not attaching or detaching anything!", action_name_.c_str());
        }
        
        return BT::NodeStatus::SUCCESS;
    }

    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s]: goal failed", action_name_.c_str());
        return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void GripperActuator::onHalted(){};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GripperActuator::providedPorts()
{
    return {BT::InputPort<double>("position"), BT::InputPort<double>("max_effort"), BT::InputPort<double>("goal_tolerance"), BT::InputPort<std::string>("attach_or_detach")};
}

#pragma endregion
