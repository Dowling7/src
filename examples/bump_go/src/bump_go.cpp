// Copyright 2025 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BumpGoNode : public rclcpp::Node
{
public:
  BumpGoNode()
  : Node("bumpgo_node"), state_(FORWARD)
  {

    // Declare the speed publisher, with the topic and queue

    // Publisher to publish velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Declare the subscriber of the laser sensor, indicating the topic and which callback function you associate it with.

    // Subscriber to laser scan data
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&BumpGoNode::laser_scan_callback, this, std::placeholders::_1));

    // Timer to run control loop
    timer_ = create_wall_timer(
      500ms, std::bind(&BumpGoNode::step, this));

    // Initialize time_stamp_ with consistent clock
    time_stamp_ = this->get_clock()->now();
  }

  // Function to handle the robot's movement logic
  void step()
  {
    geometry_msgs::msg::Twist velocity_msg;
    auto now = this->get_clock()->now(); 

    switch (state_) {
      case FORWARD:
        // Fill velocity_msg with the speeds you consider appropriate
        // IF obstacle_detected < OBSTACLE_DISTANCE:
        //   Change state to BACK;
        //   Take a timestamp of when you transition from forward to back 

        velocity_msg.linear.x = 0.2;
        velocity_msg.angular.z = 0.0;

        if (min_laser_distance_ < 0.1)  // OBSTACLE_DISTANCE
        {
          state_ = BACK;
          time_stamp_ = now;
          RCLCPP_INFO(this->get_logger(), "Obstacle detected! Switching to BACK state.");
        }
        break;

      case BACK:
        // Fill velocity_msg with the speeds you consider appropriate
        // Check if the time elapsed since the last timestamp is greater than BACKING_TIME:
        //   Change state to TURN;
        //   Take a timestamp of when you transition from back to turn

        velocity_msg.linear.x = -0.15;
        velocity_msg.angular.z = 0.0;

        if ((now - time_stamp_).seconds() > 1.5)  // BACKING_TIME
        {
          state_ = TURN;
          time_stamp_ = now;
          RCLCPP_INFO(this->get_logger(), "Switching to TURN state.");
        }
        break;

      case TURN:
        // Fill velocity_msg with the speeds you consider appropriate
        // Check if the time elapsed since the last timestamp is greater than TURNING_TIME:
        //   Change state to FORWARD;

        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.6;

        if ((now - time_stamp_).seconds() > 1.2)  // TURNING_TIME
        {
          state_ = FORWARD;
          time_stamp_ = now;
          RCLCPP_INFO(this->get_logger(), "Switching to FORWARD state.");
        }
        break;
    }

    // Publish the speed
    cmd_pub_->publish(velocity_msg);
  }

private:

  // Function to update the timestamp
  void time()
  {
    // time_stamp_ = this->now();
    time_stamp_ = this->get_clock()->now();
  }

  // Function to check if a certain duration has elapsed since the last timestamp
  bool time_elapsed(const rclcpp::Duration & duration) const
  {
    // return (this->now() - time_stamp_) > duration;
    return (this->get_clock()->now() - time_stamp_) > duration;
  }

  // Callback for laser scan messages
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    // Process the laser scan data to determine the minimum distance
    // min_laser_distance_ = (...);

    // Initialize minimum distance with the maximum possible
    float min_distance = std::numeric_limits<float>::infinity();

    // Iterate over the laser scan ranges
    for (const auto& range : msg->ranges)
    {
      if (std::isfinite(range) && range < min_distance)
      {
        min_distance = range;
      }
    }

    // Store the minimum distance in a class variable
    min_laser_distance_ = min_distance;

    RCLCPP_INFO(this->get_logger(), "Min laser distance: %.2f", min_distance);

    // Change state if an obstacle is too close
    if (min_laser_distance_ < 0.5)
    {
      state_ = BACK;
    }
    else
    {
      state_ = FORWARD;
    }
  }

  // Constants for timing
  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  // Constants for robot movement
  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.3f;
  static constexpr float OBSTACLE_DISTANCE = 0.5f;

  // State machine states
  int state_;
  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  float min_laser_distance_ = std::numeric_limits<float>::max();
  // rclcpp::Time last_state_change_time_;
  rclcpp::Time time_stamp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BumpGoNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
