#include <micro_ros_arduino.h>

#include "ball_beam_msgs/srv/in_run_control.h"

#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <ESP32Servo.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}

#define SERVO_PIN 18
#define SERVO_ZERO_POS 58

#define POT_PIN 35
#define IR_PIN 25
#define MEASURES 30

Servo base_servo;
int64_t servo_pos = 0;
int aver_pot = 0, aver_ir = 0, res_came = 0;
long unsigned int time_init = 0;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

ball_beam_msgs__srv__InRunControl_Request control_action_req;
ball_beam_msgs__srv__InRunControl_Response control_action_res;
rcl_client_t client;

#define LED 2

void blinkLed(){
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
}

void client_callback(const void * msg){
  ball_beam_msgs__srv__InRunControl_Response * msgin = (ball_beam_msgs__srv__InRunControl_Response * ) msg;

  servo_pos = msgin->servo_control_action;
  res_came = 1;
}

void setup() {
  pinMode(LED,OUTPUT);
  set_microros_transports();

  base_servo.attach(SERVO_PIN, 500, 2400);
  base_servo.setPeriodHertz(50); 
  base_servo.write(SERVO_ZERO_POS);
  
  delay(1000); 

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_ball_beam_control", "", &support));

  // create client 
  RCCHECK(rclc_client_init_default(&client, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(ball_beam_msgs, srv, InRunControl), "/ball_beam/head"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_client(&executor, &client, &control_action_res, client_callback));
   
  ball_beam_msgs__srv__InRunControl_Request__init(&control_action_req);
  time_init = millis();
}

void loop() {
  aver_pot = 0;
  aver_ir = 0;
  res_came = 0;
  for (int i = 0; i < MEASURES; i++){
    aver_pot += analogRead(POT_PIN);
    aver_ir += analogRead(IR_PIN);
  }
  aver_pot /= MEASURES;
  aver_ir /= MEASURES;

  control_action_req.pot_analog_read = aver_pot;
  control_action_req.ir_analog_read = aver_ir;
  control_action_req.time_elapsed = millis() - time_init;

  time_init = millis();
  RCCHECK(rcl_send_request(&client, &control_action_req, &servo_pos));

  while(!res_came){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(10);
    //blinkLed();
  }
  
  base_servo.write(SERVO_ZERO_POS + servo_pos);
}
