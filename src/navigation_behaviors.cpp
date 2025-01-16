#include "navigation_behaviors.h"

#pragma region GoToPose

#pragma region public methods
/**
 * @brief Construct a new Go To Pose:: Go To Pose object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
GoToPose::GoToPose(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor) : BT::StatefulActionNode(name, config)
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

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/summit/navigate_to_pose");

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
BT::NodeStatus GoToPose::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    std::string path_to_xml = package_share_directory + "/behavior_trees/";
    BT::Optional<std::string> behavior_tree_ = getInput<std::string>("behavior_tree");

    auto nav_msg = nav2_msgs::action::NavigateToPose::Goal(); 
    nav_msg.behavior_tree = path_to_xml + behavior_tree_.value();

    std::vector<std::vector<double>> deploy_coordinates_dynamic;
    getInput("deploy_coordinates_dynamic", deploy_coordinates_dynamic);

    if (deploy_coordinates_dynamic.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s]: no deploy coordinates in the list", action_name_.c_str());
    }

    else
    {
        json vec_array = deploy_coordinates_dynamic.at(0);

        std::string vec_string = vec_array.dump();

        RCLCPP_INFO(node_->get_logger(), "[%s]: going to  %s in the dynamic list", action_name_.c_str(), vec_string.c_str());
        
        // 2D pose goal
        nav_msg.pose.header.stamp = node_->get_clock()->now();
        nav_msg.pose.header.frame_id = MAP_FRAME;
        nav_msg.pose.pose.position.x = deploy_coordinates_dynamic.at(0).at(0);
        nav_msg.pose.pose.position.y = deploy_coordinates_dynamic.at(0).at(1);
        nav_msg.pose.pose.position.z = 0.0;
        nav_msg.pose.pose.orientation.x = 0.0;
        nav_msg.pose.pose.orientation.y = 0.0;
        nav_msg.pose.pose.orientation.z = 0.0;
        nav_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(node_->get_logger(), "[%s]: Sending goal: header.frame_id: %s, x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f, behavior_tree: %s", action_name_.c_str(), nav_msg.pose.header.frame_id.c_str(), nav_msg.pose.pose.position.x, nav_msg.pose.pose.position.y, nav_msg.pose.pose.position.z, nav_msg.pose.pose.orientation.x, nav_msg.pose.pose.orientation.y, nav_msg.pose.pose.orientation.z, nav_msg.pose.pose.orientation.w, nav_msg.behavior_tree.c_str());
    }

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client_->async_send_goal(nav_msg);

    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Goal was rejected by server", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus GoToPose::onRunning()
{
    // Wait for the server to be done with the goal
    auto result_future = action_client_->async_get_result(goal_handle_);

    RCLCPP_INFO(node_->get_logger(), "[%s]: Waiting for result", action_name_.c_str());

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "[%s]: Goal was aborted", action_name_.c_str());
            return BT::NodeStatus::RUNNING;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "[%s]: Goal was canceled", action_name_.c_str());
            return BT::NodeStatus::RUNNING;
        default:
            RCLCPP_ERROR(node_->get_logger(), "[%s]: Unknown result code", action_name_.c_str());
            return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s]: result received", action_name_.c_str());

    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void GoToPose::onHalted(){}

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("behavior_tree"), BT::InputPort<std::vector<std::vector<double>>>("deploy_coordinates_dynamic")};
}

#pragma endregion

#pragma endregion
