#include "rclcpp/rclcpp.hpp"
#include "ball_beam_msgs/srv/in_run_control.hpp"
#include "ball_beam_msgs/msg/in_run_monitor.hpp"
#include "ball_beam_msgs/srv/update_pid.hpp"

#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <iostream>

#define SERVO_MAX 1.5708 //from "zero" (servo=58 dg in .ino)
#define SERVO_MIN -0.959931 //from "zero" (servo=58 dg in .ino)
#define START_REF 8 //cm
#define SAMPLE_TIME 0.1 //s

using namespace std::chrono_literals;

class InRunHead : public rclcpp::Node
{
public:
    InRunHead() : Node("head")
    {
        control_service_ = this->create_service<ball_beam_msgs::srv::InRunControl>(
            "/ball_beam/head", std::bind(&InRunHead::control_callback, this, std::placeholders::_1, std::placeholders::_2));
            
        publisher_ = this->create_publisher<ball_beam_msgs::msg::InRunMonitor>("/ball_beam/monitor", 1000);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&InRunHead::timer_callback, this));

        pid_service_ = this->create_service<ball_beam_msgs::srv::UpdatePID>(
            "/ball_beam/update", std::bind(&InRunHead::update_pid_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    
private:
    // Control vars
    float ref=START_REF;
    float table_angle=0;
    int control_action=0, distance;

    // Control law vars
    float kp=1, ki=1, kd=1;
    float error=0, errorkm1=0, error_int=0, error_der=0;

    // ROS vars
    rclcpp::Service<ball_beam_msgs::srv::InRunControl>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    ball_beam_msgs::msg::InRunMonitor monitor_msg = ball_beam_msgs::msg::InRunMonitor();
    rclcpp::Publisher<ball_beam_msgs::msg::InRunMonitor>::SharedPtr publisher_;

    rclcpp::Service<ball_beam_msgs::srv::UpdatePID>::SharedPtr pid_service_;

    // Conversion functions
    int read_to_distance(int analog_read){
        return 1/(0.000035112051070*analog_read + 0.029455499904335);
    }

    float read_to_table_angle(int analog_read){
        return 0.0010037983680867989*analog_read - 2.5051757456517563;
    }

    // Control functions
    float saturate_control_action(float u){
        if (u > SERVO_MAX){
            u = SERVO_MAX;
        }
        else if (u < SERVO_MIN){
            u = SERVO_MIN;
        }
        return u;
    }

    float control_function(int dist){
        error = ref - dist;

        error_int += SAMPLE_TIME*(errorkm1+error)/2;
        error_der = (errorkm1-error)/SAMPLE_TIME;

        float u = kp*error + ki*error_int + kd*error_der;

        errorkm1 = error;

        return saturate_control_action(u);
    }

    // ROS functions
    void control_callback(const std::shared_ptr<ball_beam_msgs::srv::InRunControl::Request> request,
                          std::shared_ptr<ball_beam_msgs::srv::InRunControl::Response> response)
    {
        distance = read_to_distance(request->ir_analog_read);
        table_angle = read_to_table_angle(request->pot_analog_read);

        control_action = round((180.0/3.1415)*control_function(distance));

        response->servo_control_action = control_action;
    }
    
    void timer_callback()
    {
      monitor_msg.cur_error          = 10*error;
      monitor_msg.cur_table_angle    = table_angle;
      monitor_msg.cur_reference      = ref;
      monitor_msg.cur_distance       = distance;
      monitor_msg.cur_control_action = control_action;
      
      publisher_->publish(monitor_msg);
    }

    void update_pid_callback(const std::shared_ptr<ball_beam_msgs::srv::UpdatePID::Request> request,
                          std::shared_ptr<ball_beam_msgs::srv::UpdatePID::Response> response)
    {
        ki = request->ki;
        kp = request->kp;
        kd = request->kd;
        ref = request->new_ref;

        error = 0;
        errorkm1 = 0;
        error_int = 0;
        error_der = 0;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<InRunHead>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}