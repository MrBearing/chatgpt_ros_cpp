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

#include "chatgpt_ros_cpp/chatgpt_server.hpp"

namespace chatgpt_ros_cpp {

ChatGptServer::ChatGptServer(const rclcpp::NodeOptions& options)
  : Node("chat_gpt_server", options) {
  this->declare_parameter("api_key", "");
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  // Create the server
  server_ = this->create_service<ChatgptService>(
    "request_chat",
    std::bind(&ChatGptServer::request_chat, this, _1, _2, _3));

  // Get the API key from the ROS parameter server
  api_key_ = getApiKey();
  RCLCPP_INFO(this->get_logger(), "your api_key is \"%s\"", api_key_.c_str());

  // Log server startup message
  RCLCPP_INFO(this->get_logger(), "ChatGPTServer is ready.");
}

// ChatGptServer::~ChatGptServer() {
//   // http_client_->reset();
//   // server_->reset();
// }

void ChatGptServer::request_chat(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<ChatgptService::Request> request,
  const std::shared_ptr<ChatgptService::Response> response) {

  // set up content data
    RCLCPP_INFO(this->get_logger(), "setup request data");

  std::string query = request->question_text;
  nlohmann::json request_json;
  request_json["model"] = "gpt-3.5-turbo";
  request_json["messages"][0]["role"]= "user";
  request_json["messages"][0]["content"]= query;
  request_json["temperature"]= 0.7;

  RCLCPP_INFO(this->get_logger(), "send request object");
  web::http::http_request chatgpt_request(web::http::methods::POST);
  chatgpt_request.headers().add("Content-Type", "application/json");
  std::string auth = "Bearer " + api_key_;
  chatgpt_request.headers().add("Authorization",auth);
  chatgpt_request.set_body(request_json.dump());

  // Create the REST client
  RCLCPP_INFO(this->get_logger(), "create rest client ");
  web::http::client::http_client client(U("https://api.openai.com/v1/chat/completions"));
  web::http::http_response res;
  std::string result="";
  try {
    RCLCPP_INFO(this->get_logger(), "send request ");
    res = client.request(chatgpt_request).get();
  } catch (const std::exception & exp) {
    RCLCPP_ERROR(this->get_logger(), "Http Request Error: %s", exp.what());
    response->answer = "failed Http Request Error";
    return ;
  }

  web::json::value json_result;
  try {
    json_result = res.extract_json().get()["choices"][0]["message"]["content"];
  }catch (const std::exception & exp){
    RCLCPP_ERROR(this->get_logger(), "Parser Error: %s ",
      exp.what());
    response->answer = "failed Parser error";
    return ;
  }

  try {
    result = json_result.as_string();
  }catch (const std::exception & exp){
    RCLCPP_ERROR(this->get_logger(), "**extract Error: %s",
      exp.what() );
    response->answer = "failed extract error!";
    return ;
  }

  // Log result
  RCLCPP_INFO(this->get_logger(),
     "ChatGPT response received: %s", result.c_str());
  // Return response
  response->answer = result;
}

std::string ChatGptServer::getApiKey() {
  std::string apiKey;
  this->get_parameter_or<std::string>("api_key", apiKey, "");

  if (apiKey.empty()) {
      RCLCPP_ERROR(this->get_logger(),
      "API key not set. Please set the API key using the 'api_key' parameter");
      throw std::runtime_error("API key not set");
  }

  return apiKey;
}


}  // namespace chatgpt_ros_cpp
