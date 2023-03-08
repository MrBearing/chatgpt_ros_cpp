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

#include "chatgpt_ros/chat_gpt_action_server.hpp"

namespace chatgpt_ros {

ChatGptActionServer::ChatGptActionServer(const rclcpp::NodeOptions& options)
  : Node("chat_gpt_server", options) {
  using std::placeholders;
  // Create the action server
  mActionServer = rclcpp_action::create_server<ChatGptAct
  ion>(
    this,
    "chat_gpt",
    std::bind(&ChatGptActionServer::handleGoal, this, _1, _2),
    std::bind(&ChatGptActionServer::handleCancel, this, _1),
    std::bind(&ChatGptActionServer::handleAccepted, this, _1));

  // Create the publisher for goal status updates
  mStatusPublisher =
    this->create_publisher<action_msgs::msg::GoalStatusArray>("~/status", 10);

  // Get the API key from the ROS parameter server
  mApiKey = getApiKey();

  // Create the REST client
  mClient =
   std::make_unique<web::http::client::http_client>(
      "https://api.openai.com/v1");

  // Log server startup message
  RCLCPP_INFO(this->get_logger(), "ChatGPT server ready.");
}

void ChatGptActionServer::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ChatgptAction::Goal> goal
) {
  // Log goal request
  RCLCPP_INFO(this->get_logger(),
    "ChatGPT goal received: %s", goal->query.c_str());

  // Send the query to the API
  auto response = mClient->request(web::http::methods::POST, getApiUrl(goal->query)).get();

  // Get the response body
  std::string result = response.extract<web::json::value>().get()["choices"][0]["text"].as_string();

  // Log result
  RCLCPP_INFO(this->get_logger(), "ChatGPT response received: %s", result.c_str());

  // Send the result to the client
  auto resultMsg = ChatgptAction::Result();
  resultMsg.result = result;
  mActionServer->succeeded(uuid, resultMsg);

  // Update the goal status
  action_msgs::msg::GoalStatus goalStatus;
  goalStatus.goal_id = uuid;
  goalStatus.status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;

  action_msgs::msg::GoalStatusArray goalStatusArray;
  goalStatusArray.header.stamp = goal->header.stamp;
  goalStatusArray.status_list.push_back(goalStatus);
  mStatusPublisher->publish(goalStatusArray);
}

void ChatGptActionServer::handleCancel(
  const std::shared_ptr<GoalHandleChatGptAction> goalHandle) {
  // Log cancel request
  RCLCPP_INFO(this->get_logger(), "ChatGPT goal cancelled");

  // Notify the action server that the goal was cancelled
  mActionServer->canceled(goalHandle->get_goal_id());
}

void ChatGptActionServer::handleAccepted(
  const std::shared_ptr<GoalHandleChatGptAction> goalHandle) {
  // Log goal acceptance
  RCLCPP_INFO(this->get_logger(), "ChatGPT goal accepted");
}

std::string ChatGptActionServer::getApiKey() {
  std::string apiKey;
  this->get_parameter_or<std::string>("api_key", apiKey, "");

  if (apiKey.empty()) {
      RCLCPP_ERROR(this->get_logger(),
      "API key not set. Please set the API key using the 'api_key' parameter");
      throw std::runtime_error("API key not set");
  }

  return apiKey;
}

std::string ChatGptActionServer::getApiUrl(const std::string& query) {
  return "/engines/davinci-codex/completions?prompt="
    + web::uri::encode_data_string(query)
    + "&max_tokens=150&stop=\n&temperature=0.5";
}
}  // namespace chatgpt_ros
