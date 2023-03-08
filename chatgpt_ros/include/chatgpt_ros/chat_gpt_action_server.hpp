// Copyright 2022 Takumi Okamoto.
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

#pragma once

#include <memory>
#include <string>
#include <cpprest/http_client.h>
#include <cpprest/json.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <chatgpt_ros_msgs/action/chatgpt_action.hpp>

namespace chatgpt_ros {

class ChatGptActionServer : public rclcpp::Node {
 public:
  ChatGptActionServer();
  ~ChatGptActionServer();

 private:
  void handleGoal();
  void handleCancel();
  void sendResult(const std::string& result);
  std::string getApiUrl(const std::string& query);
  std::string getApiKey();

  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr
    mStatusPublisher;

  rclcpp::Publisher<chatgpt_ros_msgs::action::ChatgptAction::Result>
    ::SharedPtr mResultPublisher;

  rclcpp::Subscription<chatgpt_ros_msgs::action::ChatgptAction::Goal>
  ::SharedPtr mGoalSubscriber;

  rclcpp_action::Server<chatgpt_ros_msgs::action::ChatgptAction>::SharedPtr
     mActionServer;

  std::unique_ptr<web::http::client::http_client> mHttpClient;
  std::string mApiKey;
};

}  // namespace chatgpt_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(chatgpt_ros::ChatGptActionServer)
