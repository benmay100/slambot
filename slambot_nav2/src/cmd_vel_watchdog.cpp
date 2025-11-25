//
// CmdVelWatchdog listens to Nav2's action status and `/cmd_vel_nav` output.
// Once navigation goals are complete it enforces zero velocity, cutting off
// the lingering angular commands (“zombie spin”) observed on the real robot.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace slambot_nav2
{

class CmdVelWatchdog : public rclcpp::Node
{
public:
  CmdVelWatchdog()
  : rclcpp::Node("cmd_vel_watchdog"),
    goal_active_(false),
    stop_active_(false),
    last_stop_time_(this->get_clock()->now()),
    last_log_time_(this->get_clock()->now())
  {
    using namespace std::chrono_literals;

    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel_nav");
    status_topics_ = declare_parameter<std::vector<std::string>>(
      "status_topics",
      {"/navigate_to_pose/_action/status", "/navigate_through_poses/_action/status"});
    linear_threshold_ = declare_parameter<double>("linear_threshold", 0.01);
    angular_threshold_ = declare_parameter<double>("angular_threshold", 0.01);
    hold_stop_duration_ = declare_parameter<double>("hold_stop_duration", 0.5);
    log_throttle_seconds_ = declare_parameter<double>("log_throttle_seconds", 2.0);
    timer_period_seconds_ = declare_parameter<double>("timer_period", 0.1);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, qos);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, qos,
      std::bind(&CmdVelWatchdog::cmdVelCallback, this, std::placeholders::_1));

    for (const auto & topic : status_topics_) {
      auto sub = create_subscription<action_msgs::msg::GoalStatusArray>(
        topic, rclcpp::QoS(20).reliable(),
        [this, topic](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
          this->statusCallback(topic, msg);
        });
      status_subscriptions_.push_back(sub);
      action_active_[topic] = false;
    }

    stop_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_seconds_),
      std::bind(&CmdVelWatchdog::timerCallback, this));
  }

private:
  void statusCallback(
    const std::string & topic,
    action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
  {
    static const std::vector<uint8_t> active_states = {
      action_msgs::msg::GoalStatus::STATUS_ACCEPTED,
      action_msgs::msg::GoalStatus::STATUS_EXECUTING
    };

    bool active = false;
    for (const auto & status : msg->status_list) {
      if (std::find(active_states.begin(), active_states.end(), status.status) != active_states.end()) {
        active = true;
        break;
      }
    }

    action_active_[topic] = active;
    goal_active_ = std::any_of(
      action_active_.cbegin(), action_active_.cend(),
      [](const auto & entry) { return entry.second; });

    if (goal_active_) {
      stop_active_ = false;
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    if (goal_active_) {
      return;
    }

    const bool linear_trigger =
      (std::fabs(msg->linear.x) > linear_threshold_) ||
      (std::fabs(msg->linear.y) > linear_threshold_) ||
      (std::fabs(msg->linear.z) > linear_threshold_);

    const bool angular_trigger =
      (std::fabs(msg->angular.x) > angular_threshold_) ||
      (std::fabs(msg->angular.y) > angular_threshold_) ||
      (std::fabs(msg->angular.z) > angular_threshold_);

    if (linear_trigger || angular_trigger) {
      triggerStop("cmd_vel_nav drift detected");
    }
  }

  void triggerStop(const std::string & reason)
  {
    const auto now = this->get_clock()->now();
    const bool should_log =
      (now - last_log_time_).seconds() >= log_throttle_seconds_;

    if (should_log) {
      RCLCPP_WARN(get_logger(), "cmd_vel_watchdog: %s. Publishing zero twist.", reason.c_str());
      last_log_time_ = now;
    }

    stop_active_ = true;
    last_stop_time_ = now;
    publishZeroTwist();
  }

  void timerCallback()
  {
    if (!stop_active_) {
      return;
    }

    const auto now = this->get_clock()->now();
    if ((now - last_stop_time_).seconds() <= hold_stop_duration_) {
      publishZeroTwist();
    } else {
      stop_active_ = false;
    }
  }

  void publishZeroTwist()
  {
    geometry_msgs::msg::Twist zero_twist;
    cmd_vel_pub_->publish(zero_twist);
  }

  std::string cmd_vel_topic_;
  std::vector<std::string> status_topics_;
  double linear_threshold_;
  double angular_threshold_;
  double hold_stop_duration_;
  double log_throttle_seconds_;
  double timer_period_seconds_;

  bool goal_active_;
  bool stop_active_;
  rclcpp::Time last_stop_time_;
  rclcpp::Time last_log_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> status_subscriptions_;
  std::unordered_map<std::string, bool> action_active_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
};

}  // namespace slambot_nav2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slambot_nav2::CmdVelWatchdog>());
  rclcpp::shutdown();
  return 0;
}
