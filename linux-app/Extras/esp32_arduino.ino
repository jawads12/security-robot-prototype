#include <JrkG2.h>

// Define the serial ports to communicate with the Jrk G2 motor drivers
HardwareSerial jrkSerial1(1); // Using UART1
HardwareSerial jrkSerial2(2); // Using UART2

// Initialize the JrkG2Serial objects for both motor drivers
JrkG2Serial jrk1(jrkSerial1);
JrkG2Serial jrk2(jrkSerial2);

// Global variables for motor speeds
int left_motor_speed;
int right_motor_speed;

void setup() {
  // Initialize the serial ports with appropriate TX and RX pins for both motor drivers
  jrkSerial1.begin(9600, SERIAL_8N1, 16, 17); // Motor 1: RX=16, TX=17 (UART1)
  jrkSerial2.begin(9600, SERIAL_8N1, 18, 19); // Motor 2: RX=18, TX=19 (UART2)
  delay(1000); // Allow time for the motor controllers to initialize

  // Start serial monitor for debugging on UART0
  Serial.begin(115200);
  Serial.println("Setup complete");
}

void loop() {
  // Set motor speeds to 50% forward
  left_motor_speed = 3072;  // 50% forward speed
  right_motor_speed = 3072; // 50% forward speed
  run(2048, 2048);

  delay(2000); // Run at 50% speed for 2 seconds

  // // Set motor speeds to 50% reverse
  // left_motor_speed = 1024;  // 50% reverse speed
  // right_motor_speed = 1024; // 50% reverse speed
  // run(left_motor_speed, right_motor_speed);

  // delay(2000); // Run at -50% speed for 2 seconds
}

void run(int left_motor_speed, int right_motor_speed) {
  setMotorSpeed(jrk1, left_motor_speed);
  setMotorSpeed(jrk2, right_motor_speed);
}

void setMotorSpeed(JrkG2Serial &jrk, int speed) {
  if (speed > 4095) speed = 4095; // Clamp the speed value to the max
  if (speed < 0) speed = 0; // Clamp the speed value to the min
  
  Serial.print("Setting motor speed to ");
  Serial.println(speed);

  jrk.setTarget(speed);
}