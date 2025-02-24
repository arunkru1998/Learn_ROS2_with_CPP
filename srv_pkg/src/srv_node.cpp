// Including the necessary header files
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

// included to use shared pointers
#include <memory>

// create alias names for making the code more readable
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
    ServerNode() : Node("service_moving"), moving_(false)
    {
        // Create service and publisher
        // The service is binded to moving_callback and here we palce two placeholders as a service consists of request and response
        srv_ = this->create_service<SetBool>("move_clockwise", std::bind(&ServerNode::moving_callback, this, _1, _2));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Create a timer to publish velocity commands constantly in a span of 100 milliseconds
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ServerNode::publish_velocity, this));
    }

private:
// Declaring the variables and functions
    rclcpp::Service<SetBool>::SharedPtr srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist current_message_;
    bool moving_;

    void moving_callback(const std::shared_ptr<SetBool::Request> request, const std::shared_ptr<SetBool::Response> response)
    {
        if (request->data)
        {
            current_message_.linear.x = 0.5;  // Linear velocity
            current_message_.angular.z = 0.2; // Angular velocity
            moving_ = true;

            response->success = true;
            response->message = "Turning the turtle clockwise!";
        }
        else
        {
            current_message_.linear.x = 0.0;  // Stop linear velocity
            current_message_.angular.z = 0.0; // Stop angular velocity
            moving_ = false;

            response->success = true; // Indicating successful stop
            response->message = "It is time to stop the turtle!";
        }
    }

    void publish_velocity()
    {
        if (moving_)
        {
            publisher_->publish(current_message_);
        }
    }
};

int main(int argc, char *argv[])
{
    // Initialize ROS2 Client library
    rclcpp::init(argc, argv);
    // Create an instance of the class and runs it continuously
    rclcpp::spin(std::make_shared<ServerNode>());
    // Shutdown the ROS2 client library
    rclcpp::shutdown();
    return 0;
}
