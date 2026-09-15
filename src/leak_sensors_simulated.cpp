#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

class LeakSensorsSimulated : public rclcpp::Node
{
public:
  LeakSensorsSimulated()
  : Node("leak_sensors_simulated")
  {
    declare_parameter<std::string>("leak_topic", "/bluerov/stonefish/sensors/leak");
    declare_parameter<std::string>("set_leak_service", "/bluerov/stonefish/sensors/set_leak");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<std::vector<std::string>>(
      "sensor_frames",
      {"bluerov/main_cylinder", "bluerov/battery_cylinder"});
    declare_parameter<std::vector<bool>>("leak_detected", {false, false});

    const auto frames = sensor_frames();
    for (std::size_t index = 0; index < frames.size(); ++index) {
      declare_parameter<bool>("leak_detected_" + std::to_string(index), false);
    }

    const auto leak_topic = get_parameter("leak_topic").as_string();
    const auto set_leak_service = get_parameter("set_leak_service").as_string();
    auto publish_period = get_parameter("publish_period").as_double();
    if (publish_period <= 0.0) {
      publish_period = 1.0;
    }

    leak_pub_ = create_publisher<std_msgs::msg::Bool>(leak_topic, 10);
    set_leak_srv_ = create_service<std_srvs::srv::SetBool>(
      set_leak_service,
      std::bind(
        &LeakSensorsSimulated::set_leak,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period)),
      std::bind(&LeakSensorsSimulated::publish_leaks, this));
  }

private:
  bool current_leak_detected() const
  {
    const auto frames = sensor_frames();
    const auto leak_values = leak_values_for_frames(frames.size());

    auto leak_detected = false;
    for (const auto value : leak_values) {
      leak_detected = leak_detected || value;
    }

    return leak_detected;
  }

  void publish_leaks()
  {
    const auto leak_detected = current_leak_detected();

    auto msg = std_msgs::msg::Bool();
    msg.data = leak_detected;
    leak_pub_->publish(msg);
  }

  std::vector<std::string> sensor_frames() const
  {
    auto frames = get_parameter("sensor_frames").as_string_array();
    if (frames.empty()) {
      frames.emplace_back("bluerov/main_cylinder");
    }
    return frames;
  }

  std::vector<bool> leak_values_for_frames(const std::size_t frame_count) const
  {
    auto leak_values = get_parameter("leak_detected").as_bool_array();
    leak_values.resize(frame_count, false);

    for (std::size_t index = 0; index < frame_count; ++index) {
      const auto parameter_name = "leak_detected_" + std::to_string(index);
      if (has_parameter(parameter_name)) {
        leak_values[index] = get_parameter(parameter_name).as_bool();
      }
    }

    return leak_values;
  }

  void set_all_leaks(const bool leak_detected)
  {
    const auto frames = sensor_frames();
    std::vector<bool> leak_values(frames.size(), leak_detected);

    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back("leak_detected", leak_values);
    for (std::size_t index = 0; index < frames.size(); ++index) {
      parameters.emplace_back("leak_detected_" + std::to_string(index), leak_detected);
    }

    set_parameters(parameters);
  }

  void set_leak(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    set_all_leaks(request->data);
    publish_leaks();

    response->success = true;
    response->message = request->data ? "Leak simulated" : "Leak cleared";

    RCLCPP_INFO(
      get_logger(),
      "Leak simulation set to %s",
      request->data ? "true" : "false");
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr leak_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_leak_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeakSensorsSimulated>());
  rclcpp::shutdown();
  return 0;
}
