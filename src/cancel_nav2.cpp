// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

using std::placeholders::_1;

class CancelNav2 : public rclcpp::Node
{
public:
  CancelNav2()
  : Node("nav2_cancel")
  {
    // subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    //   "/summit/nav2_bt_is_active", 10, std::bind(&CancelNav2::topic_callback, this, _1));

    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/summit/nav2_bt_is_active", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&CancelNav2::topic_callback, this, _1));
    
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/summit/navigate_to_pose");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(this->get_logger(), "[nav2_cancel]: Action server not available after waiting");
    return;
    }

    timer_ = this->create_wall_timer(
      10ms, std::bind(&CancelNav2::timer_callback, this));
  }

  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    time_callback_ = this->get_clock()->now().seconds();
  }

  void timer_callback()
  {
    double time_now = this->get_clock()->now().seconds();

    if ((time_callback_ >= 0.0) && ((time_now - time_callback_) > cancel_timeout_s))
    {
      RCLCPP_WARN(this->get_logger(), "[nav2_cancel]: cancelling all nav2 action goals as BT looks inactive for the past %f seconds", cancel_timeout_s);
      
      // cancel nav2 goals
      auto cancel_goals = action_client_->async_cancel_all_goals();
      
      // auto cancel_goal_future = cancel_goals.get();

      // if (cancel_goal_future->return_code == 0)
      // {
      //   RCLCPP_WARN(this->get_logger(), "[nav2_cancel]: cancelled all nav2 action goals");
      // }

      time_callback_ = -1.0;
    } 
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  double time_callback_{-1.0};
  double cancel_timeout_s{2.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CancelNav2>());
  rclcpp::shutdown();
  return 0;
}