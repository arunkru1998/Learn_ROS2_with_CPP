#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "turtlesim/srv/set_pen.hpp"

#include <chrono>
// 1s is possible because of the using namespace std::chrono_literals
#include <cstdlib>
#include <future>
#include <memory>
#include <random>
#include <ctime>

using namespace std::chrono_literals;

class ServiceClient_turtlesim : public rclcpp::Node {
public:
  ServiceClient_turtlesim() : Node("service_client_turtlesim") {
    // Initialize two clients and a timer
    client_1 = this->create_client<std_srvs::srv::SetBool>("move_clockwise");
    client_2 = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient_turtlesim::timer_callback, this));
    service_1_done_ = false;
    service_2_done_ = false;
    // Seeds the random value with time
    std::srand(std::time(nullptr));
  }

private:
  void timer_callback() {
    // Check service status, either send the request or reset the service_called_ flag
    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      send_async_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
      service_called_ = false;
    }
  }
  // function to send the request to both the service
  void send_async_request() {
    // Cases when services fail to explained
    while ((!client_1->wait_for_service(1s))&&(!client_2->wait_for_service(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Clients interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    // auto keyword is used to automatically determine the type of the variable
    // std::make_shared is used to create a shared pointer of the instance of the class std_srvs::srv::SetBool::Request and assign it to the variable request
    auto request_1 = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto request_2 = std::make_shared<turtlesim::srv::SetPen::Request>();
    request_1->data = true;
    request_2->r = rand() % 255; // Random value between 0 and 255
    request_2->g = rand() % 255; // Random value between 0 and 255
    request_2->b = rand() % 255; // Random value between 0 and 255
    request_2->width = 5; // Width of the pen
    request_2->off = false; // Pen is on
// print the r g b colours
    RCLCPP_INFO(this->get_logger(),
                "green: %d, blue: %d, red: %d", request_2->g, request_2->b, request_2->r);
    
    // result_future is a shared pointer to the future object that will store the result of the async_send_request function
    // THe command async_send_request is used to send the request to the service and
    // the response_callback function is called when the response is received
    auto result_future_1 = client_1->async_send_request(
        request_1, std::bind(&ServiceClient_turtlesim::response_callback_1, this,
                           std::placeholders::_1));

    auto result_future_2 = client_2->async_send_request(
        request_2, std::bind(&ServiceClient_turtlesim::response_callback_2, this,
                           std::placeholders::_1));
    service_called_ = true;

    // Now check for the response after a timeout of 1 second
    auto status_1 = result_future_1.wait_for(1s);
    auto status_2 = result_future_2.wait_for(1s);
    if (status_1 != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Response 1 not ready yet.");
    }
    if (status_2 != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Response 2 not ready yet.");
    }
  }

// Callback function to check the response of the service
  void response_callback_1(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {

    // To Get response value
    auto response = future.get();
    if(response -> success){
      RCLCPP_INFO(this->get_logger(), "Response: success");
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());// convert the string to c string
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Response: failed");
    }
    service_1_done_ = true;
    check_all_services_done();// update overall service completion status
  }

  void response_callback_2(rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future) {
    RCLCPP_INFO(this->get_logger(), "Response_set_pen: success");
    service_2_done_ = true;
    check_all_services_done();
  }

  void check_all_services_done(){
    if (service_1_done_ && service_2_done_){
      service_done_=true;
    }
  }

  // initialize the public variables
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_1;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_2;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_1_done_ = false;
  bool service_2_done_ = false;
  bool service_done_ = false;
  bool service_called_ = false;

};

int main(int argc, char *argv[]) {
  // Initialize the node
  rclcpp::init(argc, argv);
  // Create an instance of the class ServiceClient_turtlesim
  auto service_client = std::make_shared<ServiceClient_turtlesim>();
 // Spin the node mutliple times callback function can be executed
  rclcpp::spin(service_client);
  rclcpp::shutdown();
  return 0;

}