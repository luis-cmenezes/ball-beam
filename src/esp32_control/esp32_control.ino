#include <micro_ros_arduino.h>

#include "ball_beam_msgs/msg/in_run_monitor.h"
#include "ball_beam_msgs/srv/update_pid.h"

#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <ESP32Servo.h>

#define LED_PIN 2

#define SERVO_PIN 18
#define SERVO_ZERO_POS 58

#define POT_PIN 35
#define IR_PIN 25
#define MEASURES 100

#define START_SETPOINT 8 //cm
#define MONITOR_TIME 150 //ms
#define SAMPLE_TIME 20 //ms
#define TOLERANCE 0.5 //cm

#define SERVO_MAX 1.5708 // rad - from "zero" (servo=58 dg)
#define SERVO_MIN -0.959931 //rad - from "zero" (servo=58 dg)

// ESP32 management vars 
Servo base_servo;
int aver_pot = 0, aver_ir = 0;

// ROS2 vars
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;
ball_beam_msgs__srv__UpdatePID_Request *pid_update_req, req;
ball_beam_msgs__srv__UpdatePID_Response *pid_update_res, res;

rcl_publisher_t publisher;
ball_beam_msgs__msg__InRunMonitor monitor_msg;
rcl_timer_t monitor_timer, control_timer;

// Control law vars
double kp=0.09, ki=0.4, kd=0.006, N=30;
int sp=START_SETPOINT, anti_windup=1;
double err=0, y=0, u=0;
double err_int=0.0,err_der,u_der;
double err_km1, u_km1;
double time_control = SAMPLE_TIME/1000.0;

float get_angle(){
  aver_pot = analogRead(POT_PIN);

  return 0.0010037983680867989*aver_pot - 2.5051757456517563;
}

float get_distance(){
  aver_ir = 0;

  for (int i = 0; i < MEASURES; i++){
    aver_ir += analogRead(IR_PIN);
  }
  aver_ir /= MEASURES;

  return 1/(0.000035112051070*aver_ir + 0.029455499904335);
}

float sature(float u){
  anti_windup=1;

  if (u > SERVO_MAX){
    u = SERVO_MAX;
    anti_windup=0;
  }else if (u < SERVO_MIN){
    u = SERVO_MIN;
    anti_windup=0;
  }

  return u;
}

void control_law(rcl_timer_t * timer, int64_t last_call_time){
  if (timer != NULL) {
    digitalWrite(LED_PIN, HIGH);

    // Read sensors
    y = get_distance();

    // Control law
    err = sp-y;
    if (abs(err) < TOLERANCE){
      err = 0;
    }

    // Integration and differentiation
    err_int += anti_windup*time_control*(err_km1+err)/2;
    err_der = (err-err_km1)/time_control;
    u_der = (u-u_km1)/time_control;

    u = kp*err + anti_windup*ki*err_int + kd*err_der - u_der/N;
    u = sature(u);

    // Apply control law
    base_servo.write(SERVO_ZERO_POS+round(180.0*u/3.14));

    //House keeping
    u_km1 = u;
    err_km1 = err;

    digitalWrite(LED_PIN, LOW);
  }
}

void publish_system_vars(rcl_timer_t * timer, int64_t last_call_time){
  if (timer != NULL) {
    monitor_msg.cur_error          = err;
    monitor_msg.cur_table_angle    = get_angle();
    monitor_msg.cur_reference      = sp;
    monitor_msg.cur_distance       = y;
    monitor_msg.cur_control_action = u;

    rcl_publish(&publisher, &monitor_msg, NULL);
  }
}

void service_callback(const void * req, void * res){
  pid_update_req = (ball_beam_msgs__srv__UpdatePID_Request *) req;
  pid_update_res = (ball_beam_msgs__srv__UpdatePID_Response *) res;
  
  kp = pid_update_req->kp;
  ki = pid_update_req->ki;
  kd = pid_update_req->kd;

  sp = pid_update_req->new_ref;
  
  err_int=0.0;

  pid_update_res->success = true;
}

void setup() {
  // Configure servo
  base_servo.attach(SERVO_PIN, 500, 2400);
  base_servo.setPeriodHertz(50); 
  base_servo.write(SERVO_ZERO_POS);
  pinMode(LED_PIN, OUTPUT);
  
  delay(1000); 

  // Configure ROS
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_control_ball_beam", "", &support);

  // Configure update PID ROS service
  rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(ball_beam_msgs, srv, UpdatePID), "/esp32/update_pid");

  // Configure monitor system ROS publisher
  rclc_publisher_init_best_effort(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(ball_beam_msgs, msg, InRunMonitor),"/esp32/monitor_system");

  // Configure monitor function timer
  rclc_timer_init_default(&monitor_timer, &support, RCL_MS_TO_NS(MONITOR_TIME), publish_system_vars);

  // Configure control law function timer
  rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(SAMPLE_TIME), control_law);

  // Start process
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &monitor_timer);
  rclc_executor_add_timer(&executor, &control_timer);
  rclc_executor_add_service(&executor, &service, &req, &res, service_callback);
}

void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}