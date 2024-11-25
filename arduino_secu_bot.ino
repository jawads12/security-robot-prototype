#include <DHT.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

// ROS Node and Publishers
ros::NodeHandle nh;
std_msgs::Float32 humidity_msg;
std_msgs::Float32 temperature_msg;
std_msgs::Float32 lpg_gas_msg;
std_msgs::Float32 co_gas_msg;
std_msgs::Float32 air_quality_msg;
std_msgs::String motion_msg;
std_msgs::String smoke_msg;

ros::Publisher pub_humidity("sensor/humidity", &humidity_msg);
ros::Publisher pub_temperature("sensor/temperature", &temperature_msg);
ros::Publisher pub_lpg_gas("sensor/lpg_gas", &lpg_gas_msg);
ros::Publisher pub_co_gas("sensor/co_gas", &co_gas_msg);
ros::Publisher pub_air_quality("sensor/air_quality", &air_quality_msg);
ros::Publisher pub_motion("sensor/motion", &motion_msg);
ros::Publisher pub_smoke("sensor/smoke", &smoke_msg);

// DHT Sensor Configuration
#define DHTPIN 2     // DHT sensor is connected to digital pin 2
#define DHTTYPE DHT11  // Change to DHT22 if you're using that model
DHT dht(DHTPIN, DHTTYPE);

// Pin Assignments for Gas Sensors and Microwave Sensor
int microwave_sensor = 4;
int mq6_sensor_pin = A3;
int mq9_sensor_pin = A4;
int mq2_sensor_pin = 3;
int mq135_sensor_pin = A5;

// Calibration Constants for Gas Sensors
float RO_MQ6 = 10000;
const float a_MQ6 = -0.45;
const float b_MQ6 = 1.43;
float RO_MQ9 = 10000;
const float a_MQ9 = -0.42;
const float b_MQ9 = 1.50;
float RO_MQ135 = 10000;
const float a_MQ135 = -0.42;
const float b_MQ135 = 1.60;

float calculatePPM(int sensorValue, float a, float b, float RO) {
  float sensor_volt = sensorValue * (5.0 / 1023.0);
  float RS_gas = (10000 * (5.0 - sensor_volt)) / sensor_volt;
  float ratio = RS_gas / RO;
  return pow(10, ((log10(ratio) - b) / a));
}

void setup() {
  Serial.begin(9600);
  pinMode(microwave_sensor, INPUT);
  pinMode(mq6_sensor_pin, INPUT);
  pinMode(mq9_sensor_pin, INPUT);
  pinMode(mq2_sensor_pin, INPUT);
  pinMode(mq135_sensor_pin, INPUT);
  dht.begin();  // Initialize the DHT sensor

  nh.initNode();
  nh.advertise(pub_humidity);
  nh.advertise(pub_temperature);
  nh.advertise(pub_lpg_gas);
  nh.advertise(pub_co_gas);
  nh.advertise(pub_air_quality);
  nh.advertise(pub_motion);
  nh.advertise(pub_smoke);
}

void loop() {
  // Read motion sensor state
  int val = digitalRead(microwave_sensor);
  static int state = LOW;
  if (val == HIGH) {
    if (state == LOW) {
      motion_msg.data = "Motion Detected";
      pub_motion.publish(&motion_msg);
      state = HIGH;
    }
  } else {
    if (state == HIGH) {
      motion_msg.data = "Motion stopped";
      pub_motion.publish(&motion_msg);
      state = LOW;
    }
  }

  // Read DHT sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (!isnan(humidity) && !isnan(temperature)) {
    humidity_msg.data = humidity;
    temperature_msg.data = temperature;
    pub_humidity.publish(&humidity_msg);
    pub_temperature.publish(&temperature_msg);
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  // Process gas sensor readings
  processGasSensors();

  nh.spinOnce();
  delay(2000);  // Delay for stability
}

void processGasSensors() {
  int mq6_value = analogRead(mq6_sensor_pin);
  int mq9_value = analogRead(mq9_sensor_pin);
  int mq2_state = digitalRead(mq2_sensor_pin);
  int mq135_value = analogRead(mq135_sensor_pin);

  if (mq2_state == HIGH) {
    smoke_msg.data = "Smoke Detected";
    pub_smoke.publish(&smoke_msg);
  } else {
    smoke_msg.data = "No Smoke Detected";
    pub_smoke.publish(&smoke_msg);
  }

  lpg_gas_msg.data = calculatePPM(mq6_value, a_MQ6, b_MQ6, RO_MQ6);
  co_gas_msg.data = calculatePPM(mq9_value, a_MQ9, b_MQ9, RO_MQ9);
  air_quality_msg.data = calculatePPM(mq135_value, a_MQ135, b_MQ135, RO_MQ135);

  pub_lpg_gas.publish(&lpg_gas_msg);
  pub_co_gas.publish(&co_gas_msg);
  pub_air_quality.publish(&air_quality_msg);
}
