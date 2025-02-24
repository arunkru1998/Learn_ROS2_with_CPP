#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
using namespace std::chrono_literals;

// Define class, which inherits from rclcpp::Node class
class pub_node : public rclcpp::Node {
public:
// Constructor for the class and the super class
  pub_node() : Node("pub_node") {
    // create publisher and timer objects within the constructor
    // Publisher for cmd_vel topic
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // Timer to publish Twist messages

    timer_ = this->create_wall_timer(
      10ms, std::bind(&pub_node::timer_callback, this));
  }

private:
  // Callback for receiving Odometry messages
  void timer_callback() {

    // Initialize Twist message
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5; // Set the linear velocity
    message.angular.z = 0.5; // Set the angular velocity
    // Publish the Twist message
    publisher_->publish(message);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  // Initialize ROS2 Client library
  rclcpp::init(argc, argv);
  // Create an instance of the class and runs it continuously
  rclcpp::spin(std::make_shared<pub_node>());
  // Shutdown the ROS2 client library
  rclcpp::shutdown();
  return 0;
}