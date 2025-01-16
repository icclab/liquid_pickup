#include "sensors_deploy_behaviors.h"

#pragma region SensorsDeploy

#pragma region public methods
/**
 * @brief Construct a new SensorsDeploy:: SensorsDeploy object
 *
 * @param name The name of the behavior
 * @param config The node configuration
 */
SensorsDeploy::SensorsDeploy(const std::string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node, const std::vector<std::vector<double>> coordinates) : BT::StatefulActionNode(name, config)
{
    action_name_ = this->name();

    if (node != nullptr)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "[%s]: Node shared pointer was passed!", action_name_.c_str());
    }
    
    RCLCPP_INFO(node_->get_logger(), "[%s]: Initialized!", action_name_.c_str());

    received_coordinates_ = coordinates;
    // RCLCPP_INFO(node_->get_logger(), "received_coordinates[0][0] %f!", received_coordinates_[0][0]);
}

/**
 * @brief method to be called at the beginning.
 *        If it returns RUNNING, this becomes an asychronous node.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SensorsDeploy::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "action start: %s", action_name_.c_str());

    setOutput<std::vector<std::vector<double>>>("deploy_coordinates", received_coordinates_);
    setOutput<std::vector<std::vector<double>>>("deploy_coordinates_dynamic", received_coordinates_);
    
    int no_of_deploy_sensors = received_coordinates_.size();

    setOutput<int>("deploy_sensors_number", no_of_deploy_sensors);
    
    RCLCPP_INFO(node_->get_logger(), "[%s]: %d sensor(s) to be deployed!", action_name_.c_str(), no_of_deploy_sensors); 
    
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief method invoked by a RUNNING action.
 *
 * @return BT::NodeStatus The status of the node
 */
BT::NodeStatus SensorsDeploy::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief when the method halt() is called and the action is RUNNING, this method is invoked.
 *        This is a convenient place todo a cleanup, if needed.
 *
 */
void SensorsDeploy::onHalted(){};

/**
 * @brief Gets the ports provided by this behavior.
 *
 * @return BT::PortsList The list of the ports
 */
BT::PortsList SensorsDeploy::providedPorts()
{
    return {BT::OutputPort<std::vector<std::vector<double>>>("deploy_coordinates"), BT::BidirectionalPort<std::vector<std::vector<double>>>("deploy_coordinates_dynamic"), BT::OutputPort<int>("deploy_sensors_number")};
}

#pragma endregion

#pragma endregion
