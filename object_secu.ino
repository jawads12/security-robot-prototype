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

// Variables to store the object avoidance data
int object_avoidance_status = 0;  // 0: No object, 8: Obstacle in front, 2: Obstacle in back

// Function prototypes
void run(int left_motor_speed, int right_motor_speed);
void setMotorSpeed(JrkG2Serial &jrk, int speed);

// Callback for receiving direction data
void directionCallback(const std_msgs::Int32& dir_msg) {
  int left_motor_speed = 2048;
  int right_motor_speed = 2048;

  // Debugging - Print the object_avoidance_status and the direction command
  Serial.print("Object avoidance status: ");
  Serial.println(object_avoidance_status);
  Serial.print("Direction command: ");
  Serial.println(dir_msg.data);

  // Forward
  if (dir_msg.data == 2) {  
    if (object_avoidance_status == 2) {  // Block forward if object detected in front
      Serial.println("Forward blocked due to obstacle in front.");
    } else {
      left_motor_speed = 4000;
      right_motor_speed = 4000;
      Serial.println("Moving forward.");
    }
  } 
  // Backward
  else if (dir_msg.data == 8) {  
    if (object_avoidance_status == 8) {  // Block backward if object detected in back
      Serial.println("Backward blocked due to obstacle in back.");
    } else {
      left_motor_speed = 500;
      right_motor_speed = 500;
      Serial.println("Moving backward.");
    }
  } 
  // Turn Right (always allowed)
  else if (dir_msg.data == 6) {  
    left_motor_speed = 4000;
    right_motor_speed = 500;
    Serial.println("Turning right.");
  } 
  // Turn Left (always allowed)
  else if (dir_msg.data == 4) {  
    left_motor_speed = 500;
    right_motor_speed = 4000;
    Serial.println("Turning left.");
  }

  run(left_motor_speed, right_motor_speed);  // Send commands to motors
}

// Callback for receiving object avoidance data
void objectAvoidanceCallback(const std_msgs::Int32& avoid_msg) {
  object_avoidance_status = avoid_msg.data;
  Serial.print("Received object avoidance status: ");
  Serial.println(object_avoidance_status);
}

// Subscriber to the direction topic
ros::Subscriber<std_msgs::Int32> sub_direction("direction", &directionCallback);

// Subscriber to the object avoidance topic
ros::Subscriber<std_msgs::Int32> sub_object_avoidance("object_avoidance", &objectAvoidanceCallback);

void setup() {
  // Initialize the serial ports for motor drivers
  Serial2.begin(115200, SERIAL_8N1);
  Serial1.begin(115200, SERIAL_8N1);

  // Start the serial communication for ROS
  Serial.begin(115200);  // Common baud rate for ROS with Teensy
  nh.initNode();        // Initialize the ROS node
  nh.subscribe(sub_direction);    // Subscribe to the 'direction' topic
  nh.subscribe(sub_object_avoidance);  // Subscribe to the 'object_avoidance' topic

  Serial.println("Setup complete.");
}

void loop() {
  nh.spinOnce();  // Handle ROS communication
  delay(90);      // Adjust delay to manage CPU load
  run(2048, 2048);  // Keep motors at a safe stop state if no command is received
}

void run(int left_motor_speed, int right_motor_speed) {
  setMotorSpeed(jrk1, left_motor_speed);
  setMotorSpeed(jrk2, right_motor_speed);
}

void setMotorSpeed(JrkG2Serial &jrk, int speed) {
  if (speed > 4095) speed = 4095;  // Clamp the speed to the maximum
  if (speed < 0) speed = 0;        // Clamp the speed to the minimum
  jrk.setTarget(speed);            // Send speed command to the motor
}
