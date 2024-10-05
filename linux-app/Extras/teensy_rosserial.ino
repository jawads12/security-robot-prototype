#include <JrkG2.h>
#include <ros.h>
#include <std_msgs/Int32.h>

// Use predefined Serial objects for Teensy
// No need to define jrkSerial1 and jrkSerial2; directly use Serial1 and Serial2

// Initialize the JrkG2Serial objects for both motor drivers
JrkG2Serial jrk1(Serial1);
JrkG2Serial jrk2(Serial2);

// Create a ROS node handle
ros::NodeHandle nh;

// Function prototypes
void run(int left_motor_speed, int right_motor_speed);
void setMotorSpeed(JrkG2Serial &jrk, int speed);

// Callback for receiving direction data
void directionCallback(const std_msgs::Int32& dir_msg) {
  if (dir_msg.data == 2) {
    run(3000, 3000);  // Adjust parameters as necessary
  }
}

// Subscriber to the direction topic
ros::Subscriber<std_msgs::Int32> sub("direction", &directionCallback);

void setup() {
  // Initialize the serial ports for motor drivers
  Serial1.begin(9600, SERIAL_8N1);
  Serial2.begin(9600, SERIAL_8N1);

  // Start the serial communication for ROS
  Serial.begin(57600);  // Common baud rate for ROS with Teensy
  nh.initNode();        // Initialize the ROS node
  nh.subscribe(sub);    // Subscribe to the 'direction' topic
}

void loop() {
  nh.spinOnce();  // Handle ROS communication
  delay(10);      // Adjust delay to manage CPU load
}

void run(int left_motor_speed, int right_motor_speed) {
  setMotorSpeed(jrk1, left_motor_speed);
  setMotorSpeed(jrk2, right_motor_speed);
}

void setMotorSpeed(JrkG2Serial &jrk, int speed) {
  if (speed > 4095) speed = 4095;  // Clamp the speed to the maximum
  if (speed < 0) speed = 0;        // Clamp the speed to the minimum
  Serial.print("Setting motor speed to ");
  Serial.println(speed);
  jrk.setTarget(speed);
}