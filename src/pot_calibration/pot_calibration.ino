#include <micro_ros_arduino.h>
#include <ball_beam_msgs/srv/potentiometer_calibration.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <ESP32Servo.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;

ball_beam_msgs__srv__PotentiometerCalibration_Request *pot_req, req;
ball_beam_msgs__srv__PotentiometerCalibration_Response *pot_res, res;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define SERVO_PIN 18
#define POT_PIN 35
#define POT_MEASURES 100

Servo base_servo;
int servo_pos = 0;
int average = 0;

void service_callback(const void * req, void * res){
  pot_req = (ball_beam_msgs__srv__PotentiometerCalibration_Request *) req;
  pot_res = (ball_beam_msgs__srv__PotentiometerCalibration_Response *) res;
  
  servo_pos += pot_req->servo_increment;
  base_servo.write(servo_pos);
  
  average = 0;

  delay(1000);

  for (int i = 0; i < POT_MEASURES; i++){
    average += analogRead(POT_PIN);
  }
  average /= POT_MEASURES;

  pot_res->pot_read = average;
}

void setup() {
  set_microros_transports();

  base_servo.attach(SERVO_PIN, 500, 2400);
  base_servo.setPeriodHertz(50); 
  base_servo.write(servo_pos);
  
  delay(1000); 

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_pot_calibration", "", &support));

  // create service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(ball_beam_msgs, srv, PotentiometerCalibration), "/potentiometer/req_move_and_read"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
