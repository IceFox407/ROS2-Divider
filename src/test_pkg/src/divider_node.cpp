#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

class DividerNode : public rclcpp::Node
{
public:
  DividerNode() : Node("divider_node")
  {
    // Creating Subscriber for /input_numbers
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/input_numbers",
      10,
      std::bind(&DividerNode::onNumbersReceived, this, std::placeholders::_1)
    );

    // Creating Publisher for /division_result
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/devision_result",
      10
    );

    RCLCPP_INFO(this->get_logger(), "DividerNode started . . .");
  }

private:
  void onNumbersReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Checking if there are 2 numbers given
    if (msg->data.size() < 2)
    {
      RCLCPP_WARN(this->get_logger(), "Less then 2 numbers. Try again with exactly 2 numbers");
      return;
    }

    if (msg->data.size() > 2)
    {
      RCLCPP_WARN(this->get_logger(), "More then 2 numbers. Try again with exactly 2 numbers");
      return;
    }

    double numerator = msg->data[0];
    double denominator = msg->data[1];

    // Checking zero devision
    if (denominator == 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Denominator is zero, division error.");
      return;
    }

    // Calculating result
    double result = numerator / denominator;

    // Publishing result
    std_msgs::msg::Float64 output_msg;
    output_msg.data = result;
    publisher_->publish(output_msg);

    RCLCPP_INFO(this->get_logger(), "Devision result is %f", result);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DividerNode>());
  rclcpp::shutdown();
  return 0;
}
