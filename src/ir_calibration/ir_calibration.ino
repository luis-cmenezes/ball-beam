#include <stdio.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18
#define SERVO_ZERO_POS 55

#define IR_PIN 25

Servo base_servo;
int aver_measure;

void setup() {
  base_servo.attach(SERVO_PIN, 500, 2400);
  base_servo.setPeriodHertz(50); 
  base_servo.write(SERVO_ZERO_POS);

  delay(1000);

  Serial.begin(9600);
}

void loop() {

  aver_measure = 0;

  for (int i = 0; i < 30; i++){
    aver_measure += analogRead(IR_PIN);
  }

  aver_measure /= 30;

  Serial.println(aver_measure);
  delay(200);
}
