#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

// include the necessary header files for turtlesim action message
#include "turtlesim/action/rotate_absolute.hpp"
// include the header files for creating action client in ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//Create a class called TurtleActionClient
class TurtleActionClient : public rclcpp::Node
{
public:
    // create Alias for the class we are using
  using Rotate = turtlesim::action::RotateAbsolute;
  using GoalHandleRotate = rclcpp_action::ClientGoalHandle<Rotate>;
  // Constructor definition
  explicit TurtleActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("turtle_action_client", node_options), goal_done_(false)
  {
    //
    this->client_ptr_ = rclcpp_action::create_client<Rotate>(
      this->get_node_base_interface(),// returns the base interface of the node
      this->get_node_graph_interface(),// returns the graph interface of the node
      this->get_node_logging_interface(),// returns the logging interface of the node
      this->get_node_waitables_interface(),//  returns the waitable interface of the node
      "/turtle1/rotate_absolute");
    // timer
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TurtleActionClient::send_goal, this));
  }
    // check the completion of the goal
  bool is_goal_done() const
  {
    return this->goal_done_;
  }

    // responsible for sending the goal 
    // responsible for setting up the necessary callback functions
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }
    // wait for connection with action server for 10 seconds and end it not connected for 10 seconds
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    // initialize goal_msg
    auto goal_msg = Rotate::Goal();
    goal_msg.theta = 3.14;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // create goal options to initiate different callback for goal response, feedback and result
    auto send_goal_options = rclcpp_action::Client<Rotate>::SendGoalOptions();
                
    // response
    send_goal_options.goal_response_callback =
      std::bind(&TurtleActionClient::goal_response_callback, this, _1);
    // feedback
    send_goal_options.feedback_callback =
      std::bind(&TurtleActionClient::feedback_callback, this, _1, _2);
    // result_callback
    send_goal_options.result_callback =
      std::bind(&TurtleActionClient::result_callback, this, _1);
    // place we call the action server with corresponding message  
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
 // initialize variables, pointers
  rclcpp_action::Client<Rotate>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;


// turtlesim action messsage -> theta, delta, remaining
    // callback for response
  void goal_response_callback(const GoalHandleRotate::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

    // feedback
  void feedback_callback(
    GoalHandleRotate::SharedPtr,
    const std::shared_ptr<const Rotate::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(), "Feedback received Remaining angle: %f", feedback->remaining);
  }

    // result
  void result_callback(const GoalHandleRotate::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->delta);

  }
};  


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<TurtleActionClient>();
 // initialize a multithread executor to run multiple threads
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}