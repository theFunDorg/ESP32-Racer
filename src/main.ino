
// #include <micro_ros_arduino.h>


#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//  PACKAGES FOR ROS2 MESSAGES  //
#include <racer_interfaces/msg/engine_actuate.h>
#include <racer_interfaces/msg/engine_sensor.h>

//  PACKAGES TOF SENSOR  //
#include <Wire.h>
#include <VL53L0X.h>

//  PACKAGES SERVOS  //
#include <ESP32Servo.h>

// PACKAGES 6DOF  //
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// SETUP VARIABLES FOR SERVO //
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int minUs = 1000;
int maxUs = 2000;
int servo1Pin = 14;
int servo2Pin = 15;
int servo3Pin = 16;
int servo4Pin = 17;

// SETUP VARIABLES FOR TOF SENSOR //
VL53L0X tof;

// SETUP VARIABLES FOR 6DOF SENSOR //
Adafruit_MPU6050 mpu;


// SETUP VARIABLES FOR ROS2 //
rcl_publisher_t publisher_sensor;
rcl_subscription_t subscriber_actuate;

rclc_executor_t executor;

rclc_support_t support;

rcl_allocator_t allocator;

rcl_node_t node = rcl_get_zero_initialized_node();
rcl_timer_t timer_sensor;

// SETUP MESSAGE FILE TYPE
// msg__engine_actuate 
racer_interfaces__msg__EngineActuate msgIn;
racer_interfaces__msg__EngineSensor msgOut;
//#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
  //  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
 }


//twist message cb
void subscription_callback(const void *msgin) {
  const racer_interfaces__msg__EngineActuate * msgIn = (const racer_interfaces__msg__EngineActuate *)msgIn;
  servo1.write((int32_t) msgIn->l_elevon);
  servo2.write((int32_t) msgIn->r_elevon);
  servo3.write((int32_t) msgIn->v_nozzle);
  servo4.write((int32_t) msgIn->h_nozzle);
 }


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
      msgOut.height=(int32_t) tof.readRangeContinuousMillimeters();

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    msgOut.ax=(a.acceleration.x);
    msgOut.ay=(a.acceleration.y);
    msgOut.az=(a.acceleration.z);
    msgOut.gx=(g.gyro.x);
    msgOut.gy=(g.gyro.y);
    msgOut.gz=(g.gyro.z);
      
    RCSOFTCHECK(rcl_publish(&publisher_sensor, &msgOut, NULL));
  }
 }
}
// ======================================================================================================== //
// SETUP
// ======================================================================================================== //
void setup() {

  // -------------------------- SETTING UP TOF SENSOR -------------------------- //
  Wire.begin();
  tof.setTimeout(500);
  if (!tof.init())
  {
    while (1) {}
  }
  tof.startContinuous();

  // -------------------------- SETTING UP SERVO -------------------------- //
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo1.attach(servo1Pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	servo2.setPeriodHertz(50);
	servo2.attach(servo2Pin, 1000, 2000);
	servo3.setPeriodHertz(50);
	servo3.attach(servo3Pin, 1000, 2000);
	servo4.setPeriodHertz(50);
	servo4.attach(servo4Pin, 1000, 2000);


  // -------------------------- SETTING UP 6DOF SENSOR -------------------------- //

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);


  // -------------------------- SETTING UP ROS2 -------------------------- //
  set_microros_transports();
  // Add this in at a later point, comment out line above
  //static inline void set_microros_wifi_transports(char *ssid, char *pass, char *agent_ip, uint agent_port)
  //set_microros_wifi_transports("malware.exe", "WIFI PASS", "192.168.1.57", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();
  
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create nodes
  RCCHECK(rclc_node_init_default(&node, "controller", "left_engine", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_sensor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(racer_interfaces, msg, EngineSensor),
    "engine_sensors"));
        

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_actuate,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(racer_interfaces, msg, EngineActuate),
    "controls"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer_sensor,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // -------------------------- SETTING ROS EXECUTOR -------------------------- //

  // total number of handles = #subscriptions + #timers
  unsigned int num_handles = 1 + 1;
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  
  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_sensor));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_actuate, &msgIn, &subscription_callback, ON_NEW_DATA));
  // Optional prepare for avoiding allocations during spin
  RCCHECK(rclc_executor_prepare(&executor));

  //msgOut.data = 0;
  //msgIn.data = 0;

 };

// ======================================================================================================== //
// LOOP
// ======================================================================================================== //

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10000)));
 };