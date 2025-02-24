#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Define class, which inherits from rclcpp::Node class
class sub_node : public rclcpp::Node {
public:
// Constructor for the class and the super class
  sub_node() : Node("sub_node") {
    // create publisher and timer objects within the constructor
    // Publisher for cmd_vel topic
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // Timer to publish Twist messages
    subscriber_ = this->create_subscription<std_msgs::msg::String>("/direction", 10,
     std::bind(&sub_node::callback, this, std::placeholders::_1));

  }




private:
  // Callback for receiving Odometry messages
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    auto message = geometry_msgs::msg::Twist();
    if(msg->data == "clockwise"){
        RCLCPP_INFO(this->get_logger(), "Clockwise");
        message.linear.x = 0.5; // Set the linear velocity
        message.angular.z = 0.5; // Set the angular velocity
        // Publish the Twist message
        publisher_->publish(message);
    }
    else if(msg->data == "anticlockwise"){
        RCLCPP_INFO(this->get_logger(), "Anticlockwise");
        message.linear.x = 0.5; // Set the linear velocity
        message.angular.z = -0.5; // Set the angular velocity
        // Publish the Twist message
        publisher_->publish(message);
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};




int main(int argc, char *argv[]) {
  // Initialize ROS2 Client library
  rclcpp::init(argc, argv);
  // Create an instance of the class and runs it continuously
  rclcpp::spin(std::make_shared<sub_node>());
  // Shutdown the ROS2 client library
  rclcpp::shutdown();
  return 0;
}