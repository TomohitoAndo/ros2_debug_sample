// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>

using namespace std::chrono_literals;

namespace hello_world_debug
{

  class TalkerComponent : public rclcpp::Node
  {
  public:
    explicit TalkerComponent(const rclcpp::NodeOptions &options)
        : Node("talker_component", options)
    {
      auto publish_message =
          [this]() -> void
      {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello World!";

        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        pub_->publish(std::move(msg));

        count++;
        RCLCPP_INFO_STREAM(get_logger(), "count: " << count);


        if (count > 30) {
          throw std::runtime_error("error!"); 
        }
      };

      rclcpp::QoS qos(rclcpp::KeepLast(10));
      pub_ = create_publisher<std_msgs::msg::String>("chatter", qos);
      timer_ = create_wall_timer(100ms, publish_message);
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count{0};
  };

} // namespace hello_world_debug

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_debug::TalkerComponent)
