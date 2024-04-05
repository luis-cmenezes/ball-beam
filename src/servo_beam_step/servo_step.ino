#include <micro_ros_arduino.h>
#include <ball_beam_msgs/srv/trigger_step.h>
#include <ball_beam_msgs/msg/step_data.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <ESP32Servo.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 2
#define SERVO_PIN 18
#define POT_PIN 35
#define POT_MEASURES 30

#define STEP_TIME 1000 //ms
#define EXPERIMENT_TIME 5000 //ms

#define SERVO_START_POS 58

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;

ball_beam_msgs__srv__TriggerStep_Request *step_req, req;
ball_beam_msgs__srv__TriggerStep_Response res;

rcl_publisher_t publisher;
ball_beam_msgs__msg__StepData step_msg;

Servo base_servo;
int servo_pos = SERVO_START_POS, pot_read = 0;

void error_loop(){
  for(int i = 0; i < 10; i++){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

int pot_read_fn(){
  pot_read = 0;

  for (int i = 0; i < POT_MEASURES; i++){
    pot_read += analogRead(POT_PIN);
  }
  pot_read /= POT_MEASURES;

  return pot_read;
}

void step_test(){
	unsigned int start_time = millis();
	unsigned int cur_time=start_time;
	int cur_servo, cur_pot, exp_time=cur_time-start_time;

	while(exp_time <= EXPERIMENT_TIME){
		cur_time = millis();

    exp_time = cur_time-start_time;

		if(exp_time >= STEP_TIME)
			base_servo.write(servo_pos);
		else
			base_servo.write(SERVO_START_POS);
		
		cur_servo = base_servo.read();
		cur_pot = pot_read_fn();

    step_msg.time = exp_time;
    step_msg.servo = cur_servo;
    step_msg.pot_read = cur_pot;

    RCSOFTCHECK(rcl_publish(&publisher, &step_msg, NULL));
	}
  
  base_servo.write(SERVO_START_POS);
  servo_pos = SERVO_START_POS;
}

void service_callback(const void * req, void * res){
  step_req = (ball_beam_msgs__srv__TriggerStep_Request *) req;
  
  servo_pos += step_req->servo_step;
  
  step_test();
}

void setup() {
	pinMode(LED_PIN,OUTPUT);

	base_servo.attach(SERVO_PIN, 500, 2400);
	base_servo.setPeriodHertz(50); 
	base_servo.write(servo_pos);
	
	delay(1000);

	set_microros_transports();

	allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "esp32_step_response", "", &support));

	// create service
	RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(ball_beam_msgs, srv, TriggerStep), "/step/req_new_exp"));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(ball_beam_msgs, msg, StepData),
		"/step/data"));

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

	RCCHECK(rclc_executor_spin(&executor));
}

void loop() {
}