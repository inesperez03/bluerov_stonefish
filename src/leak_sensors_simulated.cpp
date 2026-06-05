#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class LeakSensorsSimulated : public rclcpp::Node
{
public:
  LeakSensorsSimulated()
  : Node("leak_sensors_simulated")
  {
    declare_parameter<std::string>("leak_topic", "/bluerov/stonefish/sensors/leak");
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
    auto publish_period = get_parameter("publish_period").as_double();
    if (publish_period <= 0.0) {
      publish_period = 1.0;
    }

    leak_pub_ = create_publisher<std_msgs::msg::Float32>(leak_topic, 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period)),
      std::bind(&LeakSensorsSimulated::publish_leaks, this));
  }

private:
  void publish_leaks()
  {
    const auto frames = sensor_frames();
    const auto leak_values = leak_values_for_frames(frames.size());

    auto leak_detected = false;
    for (const auto value : leak_values) {
      leak_detected = leak_detected || value;
    }

    auto msg = std_msgs::msg::Float32();
    msg.data = leak_detected ? 1.0F : 0.0F;
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

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leak_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeakSensorsSimulated>());
  rclcpp::shutdown();
  return 0;
}
