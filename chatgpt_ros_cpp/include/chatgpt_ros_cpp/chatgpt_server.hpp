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
#include <cpprest/filestream.h>
#include <cpprest/json.h>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <chatgpt_ros_interface/srv/chatgpt_service.hpp>

namespace chatgpt_ros_cpp {

using ChatgptService = chatgpt_ros_interface::srv::ChatgptService;

class ChatGptServer : public rclcpp::Node{
 public:
  ChatGptServer() = delete;
  explicit ChatGptServer(const rclcpp::NodeOptions &);
  // ~ChatGptServer();

 private:
  std::string api_key_;
  rclcpp::Service<ChatgptService>::SharedPtr server_;

  std::string getApiKey();

  void request_chat(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ChatgptService::Request> request,
    const std::shared_ptr<ChatgptService::Response> response);
};

}  // namespace chatgpt_ros_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(chatgpt_ros_cpp::ChatGptServer)
