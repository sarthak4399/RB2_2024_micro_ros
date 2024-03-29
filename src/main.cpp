
#include <Arduino.h>
#include <Wire.h>                    // Wire library for I2C communication
#include <ros.h>                     // ROS library
#include <std_msgs/Float32MultiArray.h> // ROS standard message type
#include <std_msgs/Float32.h>        // ROS standard message type

// ROS node handle
ros::NodeHandle nh;

// ROS message
std_msgs::Float32MultiArray msg;

// Motor Pins
const int numMotors = 4;
int DIR_PINS[numMotors] = {13, 14, 26, 33};
int PWM_PINS[numMotors] = {12, 27, 25, 32};

class MotorControl {

public:
  // Constructor
  MotorControl() {
    // Initialize motor pins
    for (int i = 0; i < numMotors; ++i) {
      pinMode(DIR_PINS[i], OUTPUT);
      pinMode(PWM_PINS[i], OUTPUT);
    }
  }


  // Function to set motor speed and direction
  void setMotorSpeed(int motorIndex, float speed) {
    // Ensure motor index is valid
    if (motorIndex >= 0 && motorIndex < numMotors) {
      // Set direction
      if (speed >= 0) {
        digitalWrite(DIR_PINS[motorIndex], HIGH); // Set direction forward
      } else {
        digitalWrite(DIR_PINS[motorIndex], LOW);  // Set direction backward
      }
      // Set PWM value for speed control
      analogWrite(PWM_PINS[motorIndex], abs(speed));
    }
  }

  // Function to stop all motors
  void stopAllMotors() {
    for (int i = 0; i < numMotors; ++i) {
      digitalWrite(DIR_PINS[i], LOW);   // Stop motor
      analogWrite(PWM_PINS[i], 0);       // Set PWM to 0
    }
  }
};

void Kinematics_callback(const std_msgs::Float32MultiArray &msg) {
  // Creating motor control object
  static
  MotorControl MotorControl;
  // Get wheel velocities from message
  float v1 = msg.data[0];
  float v2 = msg.data[1];
  float v3 = msg.data[2];
  float v4 = msg.data[3];

  for (int i = 0; i < numMotors; ++i) {
    Serial.print("{ Motor");
    Serial.print(i + 1);
    Serial.print("Velocity") ;
    Serial.print(" : ");
    Serial.print(msg.data[i]);
    Serial.print(" }");
    if (i < numMotors - 1) {
      Serial.print(" , ");
    }
  }
  Serial.println();
  // Calculate motor speeds
  float w1 = (v1 + v2 + v3 + v4) / 4;
  float w2 = (-v1 + v2 + v3 - v4) / 4;
  float w3 = (-v1 + v2 - v3 + v4) / 4;
  float w4 = (v1 + v2 - v3 - v4) / 4;

  
  // Set motor speeds
  MotorControl.setMotorSpeed(0, w1);
  MotorControl.setMotorSpeed(1, w2);
  MotorControl.setMotorSpeed(2, w3);
  MotorControl.setMotorSpeed(3, w4);
}
// ROS subscriber
ros::Subscriber<std_msgs::Float32MultiArray> sub("/whell_vel", &Kinematics_callback);

void setup() {
  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
