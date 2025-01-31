#define ROS_SERIAL_BUFFER_SIZE 2048  // Increase buffer size for larger messages

#include <ros.h>
#include <SabertoothSimplified.h>
#include <std_msgs/Int32.h>

// Initialize the Sabertooth motor driver
#define ST_ADDRESS 128 // Sabertooth address (default: 128)
#define MOTOR1 1       // Sabertooth motor 1 (right)
#define MOTOR2 2       // Sabertooth motor 2 (left)

SabertoothSimplified ST(Serial1);

// Create a ROS node handle
ros::NodeHandle nh;

// Variables to store the object avoidance data
int object_avoidance_status = 0;  // 0: No object, 8: Obstacle in front, 2: Obstacle in back

// Function prototypes
void run(int left_motor_speed, int right_motor_speed);
void setMotorSpeed(int motor, int speed);

// Callback for receiving direction data
void directionCallback(const std_msgs::Int32& dir_msg) {
  int left_motor_speed = 0;
  int right_motor_speed = 0;

  // Debugging - Print the object_avoidance_status and the direction command
//  Serial.print("Object avoidance status: ");
//  Serial.println(object_avoidance_status);
//  Serial.print("Direction command: ");
//  Serial.println(dir_msg.data);

  // backward
  if (dir_msg.data == 2) {  
    if (object_avoidance_status == 2) {  // Block forward if object detected in front
    //  Serial.println("Forward blocked due to obstacle in front.");
    } else {

      left_motor_speed = -30;
    right_motor_speed = 30;
     
      
     // Serial.println("Moving forward.");
    }
  } 
  // forward
  else if (dir_msg.data == 8) {  
    if (object_avoidance_status == 8) {  // Block backward if object detected in back
     // Serial.println("Backward blocked due to obstacle in back.");
    } else {


      left_motor_speed = 30;
      right_motor_speed = -30;
      
     // Serial.println("Moving backward.");
    }
  } 
  // Turn Right (always allowed)
  else if (dir_msg.data == 6) {  
    left_motor_speed = 30;
    right_motor_speed = 30;
    //Serial.println("Turning right.");
  } 
  // Turn Left (always allowed)
  else if (dir_msg.data == 4) {  
     left_motor_speed = -30;
     right_motor_speed = -30;
    
   // Serial.println("Turning left.");
  }

 

  run(left_motor_speed, right_motor_speed);  // Send commands to motors
}

// Callback for receiving object avoidance data
void objectAvoidanceCallback(const std_msgs::Int32& avoid_msg) {
  object_avoidance_status = avoid_msg.data;
 // Serial.print("Received object avoidance status: ");
 // Serial.println(object_avoidance_status);
}

// Subscriber to the direction topic
ros::Subscriber<std_msgs::Int32> sub_direction("direction", &directionCallback);

// Subscriber to the object avoidance topic
ros::Subscriber<std_msgs::Int32> sub_object_avoidance("object_avoidance", &objectAvoidanceCallback);

void setup() {
  // Initialize the serial port for Sabertooth communication
  Serial1.begin(9600); // Communication with Sabertooth

  // Start the serial communication for ROS
  Serial.begin(115200);  // Common baud rate for ROS with Teensy
  nh.initNode();        // Initialize the ROS node
  nh.subscribe(sub_direction);    // Subscribe to the 'direction' topic
  nh.subscribe(sub_object_avoidance);  // Subscribe to the 'object_avoidance' topic

 // Serial.println("Setup complete.");
}

void loop() {
  nh.spinOnce();  // Handle ROS communication
  delay(120);      // Adjust delay to manage CPU load
  run(0, 0);  // Keep motors at a safe stop state if no command is received
}

void run(int left_motor_speed, int right_motor_speed) {
  setMotorSpeed(MOTOR1, left_motor_speed);
  setMotorSpeed(MOTOR2, right_motor_speed);
}

void setMotorSpeed(int motor, int speed) {
  speed = constrain(speed, -100, 100);  // Clamp the speed to the valid range
  ST.motor(motor, speed);             // Send speed command to the motor
}
