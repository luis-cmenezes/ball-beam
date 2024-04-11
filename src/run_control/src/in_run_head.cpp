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
    int ref=START_REF, control_action=0;
    float table_angle=0.0, distance=0.0;

    // ROS vars
    rclcpp::Service<ball_beam_msgs::srv::InRunControl>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    ball_beam_msgs::msg::InRunMonitor monitor_msg = ball_beam_msgs::msg::InRunMonitor();
    rclcpp::Publisher<ball_beam_msgs::msg::InRunMonitor>::SharedPtr publisher_;

    rclcpp::Service<ball_beam_msgs::srv::UpdatePID>::SharedPtr pid_service_;

    // Control law vars
    float error, err_der, err_int=0.0, err_km1=0.0;
    float u=0.0, u_km1=0.0, u_der;
    float dt;
    float kp=0.07, ki=0, kd=0;
    int anti_windup=1, N=30;

    // Conversion functions
    float read_to_distance(int analog_read){
        return 1/(0.000035112051070*analog_read + 0.029455499904335);
    }

    float read_to_table_angle(int analog_read){
        return 0.0010037983680867989*analog_read - 2.5051757456517563;
    }

    // Control functions
    float saturate_control_action(float u){
        anti_windup=1;
        if (u > SERVO_MAX){
            u = SERVO_MAX;
            anti_windup=0;
        }
        else if (u < SERVO_MIN){
            u = SERVO_MIN;
            anti_windup=0;
        }

        return u;
    }

    float control_function(float y, float dt){
        error = ref - y;
        err_der = (error-err_km1)/dt;
        err_int += anti_windup*dt*(err_km1+error)/2;

        u_der = (u-u_km1)/dt;

        u = kp*error + anti_windup*ki*err_int + kd*err_der - u_der/N;
        u = saturate_control_action(u);

        err_km1 = error;
        u_km1 = u;

        return u;
    }

    // ROS functions
    void control_callback(const std::shared_ptr<ball_beam_msgs::srv::InRunControl::Request> request,
                          std::shared_ptr<ball_beam_msgs::srv::InRunControl::Response> response)
    {   
        //std::cout << "Request: " << request->ir_analog_read << " " << request->time_elapsed << "\n";

        distance = read_to_distance(request->ir_analog_read);
        dt = request->time_elapsed/1E3;

        control_action = round((180.0/3.1415)*control_function(distance, dt));

        response->servo_control_action = control_action;

        //std::cout << "Response: " << response->servo_control_action << "\n\n";

        table_angle = read_to_table_angle(request->pot_analog_read);
    }
    
    void timer_callback()
    {
      monitor_msg.cur_error          = error;
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

        err_int = 0;
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