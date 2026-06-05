#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

class DvlToTwistNode : public rclcpp::Node
{
public:
  DvlToTwistNode()
  : Node("dvl_to_twist_node")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/bluerov/stonefish/sensors/dvl_sim");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/bluerov/stonefish/sensors/dvl");

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic, rclcpp::SensorDataQoS());

    subscription_ = create_subscription<stonefish_ros2::msg::DVL>(
      input_topic,
      rclcpp::SensorDataQoS(),
      [this](const stonefish_ros2::msg::DVL::SharedPtr msg) {
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header = msg->header;
        twist_msg.twist.linear = msg->velocity;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        publisher_->publish(twist_msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Converting DVL messages from '%s' to TwistStamped on '%s'",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlToTwistNode>());
  rclcpp::shutdown();
  return 0;
}
