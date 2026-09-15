#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class BatteryStatusSimulated : public rclcpp::Node
{
public:
  BatteryStatusSimulated()
  : Node("battery_status_simulated")
  {
    declare_parameter<std::string>("battery_topic", "/bluerov/stonefish/sensors/battery");
    declare_parameter<std::vector<std::string>>(
      "thruster_topics",
      {"/bluerov/controller/thruster_setpoints_sim"});
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("nominal_voltage", 16.8);
    declare_parameter<double>("voltage_oscillation_amplitude", 0.15);
    declare_parameter<double>("voltage_oscillation_period", 12.0);
    declare_parameter<int>("cell_count", 4);
    declare_parameter<double>("full_cell_voltage", 4.2);
    declare_parameter<double>("empty_cell_voltage", 3.25);
    declare_parameter<double>("design_capacity_ah", 16.0);
    declare_parameter<double>("jetson_current_a", 2.0);
    declare_parameter<double>("base_electronics_current_a", 0.5);
    declare_parameter<double>("current_per_thruster_unit_a", 8.0);
    declare_parameter<double>("max_thruster_command", 1.0);
    declare_parameter<double>("stale_thruster_timeout", 2.0);
    declare_parameter<std::string>("battery_frame_id", "bluerov/battery");

    const auto battery_topic = get_parameter("battery_topic").as_string();
    auto publish_period = get_parameter("publish_period").as_double();
    if (publish_period <= 0.0) {
      publish_period = 1.0;
    }

    start_time_seconds_ = now_seconds();
    last_update_seconds_ = start_time_seconds_;
    remaining_charge_ah_ = charge_from_voltage(get_parameter("nominal_voltage").as_double());
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&BatteryStatusSimulated::on_parameters_set, this, std::placeholders::_1));
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, 10);

    auto thruster_topics = get_parameter("thruster_topics").as_string_array();
    if (thruster_topics.empty()) {
      thruster_topics.emplace_back("/bluerov/controller/thruster_setpoints_sim");
    }

    for (const auto & topic : thruster_topics) {
      thrusters_.emplace(topic, ThrusterTopicState{});
      subscriptions_.push_back(create_subscription<std_msgs::msg::Float64MultiArray>(
        topic,
        10,
        [this, topic](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          auto & state = thrusters_[topic];
          state.commands = msg->data;
          state.last_update_seconds = now_seconds();
        }));
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period)),
      std::bind(&BatteryStatusSimulated::publish_battery, this));
  }

private:
  struct ThrusterTopicState
  {
    std::vector<double> commands;
    double last_update_seconds = -1.0;
  };

  void publish_battery()
  {
    const auto stamp = get_clock()->now();
    const auto seconds = stamp.seconds();
    const auto total_current =
      get_parameter("jetson_current_a").as_double() +
      get_parameter("base_electronics_current_a").as_double() +
      estimate_thruster_current(seconds);

    update_discharge(seconds, total_current);

    const auto design_capacity = std::max(get_parameter("design_capacity_ah").as_double(), 1e-6);
    const auto percentage = std::clamp(remaining_charge_ah_ / design_capacity, 0.0, 1.0);
    const auto voltage = battery_voltage(seconds, percentage);
    const auto cell_count = std::max(get_parameter("cell_count").as_int(), 1L);
    const auto cell_voltage = voltage / static_cast<double>(cell_count);

    auto msg = sensor_msgs::msg::BatteryState();
    msg.header.stamp = stamp;
    msg.header.frame_id = get_parameter("battery_frame_id").as_string();
    msg.voltage = voltage;
    msg.current = -total_current;
    msg.charge = percentage * design_capacity;
    msg.capacity = design_capacity;
    msg.design_capacity = design_capacity;
    msg.percentage = percentage;
    msg.present = true;
    msg.cell_voltage.assign(static_cast<std::size_t>(cell_count), cell_voltage);
    msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = battery_health(percentage);
    msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    battery_pub_->publish(msg);
  }

  double battery_voltage(const double seconds, const double percentage) const
  {
    const auto amplitude = get_parameter("voltage_oscillation_amplitude").as_double();
    const auto period = std::max(
      get_parameter("voltage_oscillation_period").as_double(), 1e-6);
    const auto elapsed = seconds - start_time_seconds_;
    constexpr auto pi = 3.14159265358979323846;
    return voltage_from_percentage(percentage) + amplitude * std::sin(2.0 * pi * elapsed / period);
  }

  double voltage_from_percentage(const double percentage) const
  {
    const auto cell_count = std::max(get_parameter("cell_count").as_int(), 1L);
    const auto full_cell_voltage = get_parameter("full_cell_voltage").as_double();
    const auto empty_cell_voltage = get_parameter("empty_cell_voltage").as_double();
    const auto clamped_percentage = std::clamp(percentage, 0.0, 1.0);
    const auto cell_voltage =
      empty_cell_voltage + clamped_percentage * (full_cell_voltage - empty_cell_voltage);
    return cell_voltage * static_cast<double>(cell_count);
  }

  double charge_from_voltage(const double voltage) const
  {
    const auto cell_count = std::max(get_parameter("cell_count").as_int(), 1L);
    const auto cell_voltage = voltage / static_cast<double>(cell_count);
    const auto full_cell_voltage = get_parameter("full_cell_voltage").as_double();
    const auto empty_cell_voltage = get_parameter("empty_cell_voltage").as_double();
    const auto usable_range = std::max(full_cell_voltage - empty_cell_voltage, 1e-6);
    const auto percentage = (cell_voltage - empty_cell_voltage) / usable_range;
    const auto design_capacity = std::max(get_parameter("design_capacity_ah").as_double(), 1e-6);
    return std::clamp(percentage, 0.0, 1.0) * design_capacity;
  }

  void update_discharge(const double seconds, const double total_current)
  {
    const auto elapsed_hours = std::max(seconds - last_update_seconds_, 0.0) / 3600.0;
    remaining_charge_ah_ = std::max(remaining_charge_ah_ - total_current * elapsed_hours, 0.0);
    last_update_seconds_ = seconds;
  }

  std::uint8_t battery_health(const double percentage) const
  {
    if (percentage < 0.25) {
      return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    }
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  }

  double estimate_thruster_current(const double seconds) const
  {
    auto total_normalized_effort = 0.0;
    const auto max_thruster_command = std::max(
      get_parameter("max_thruster_command").as_double(), 1e-6);
    const auto stale_thruster_timeout =
      get_parameter("stale_thruster_timeout").as_double();

    for (const auto & entry : thrusters_) {
      const auto & state = entry.second;
      if (state.last_update_seconds < 0.0) {
        continue;
      }
      if (seconds - state.last_update_seconds > stale_thruster_timeout) {
        continue;
      }

      for (const auto command : state.commands) {
        total_normalized_effort += std::min(std::abs(command) / max_thruster_command, 1.0);
      }
    }

    return total_normalized_effort *
      get_parameter("current_per_thruster_unit_a").as_double();
  }

  double now_seconds()
  {
    return get_clock()->now().seconds();
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "nominal_voltage" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        remaining_charge_ah_ = charge_from_voltage(parameter.as_double());
        start_time_seconds_ = now_seconds();
        last_update_seconds_ = start_time_seconds_;
      }
    }

    return result;
  }

  double start_time_seconds_ = 0.0;
  double last_update_seconds_ = 0.0;
  double remaining_charge_ah_ = 0.0;
  std::unordered_map<std::string, ThrusterTopicState> thrusters_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> subscriptions_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStatusSimulated>());
  rclcpp::shutdown();
  return 0;
}
