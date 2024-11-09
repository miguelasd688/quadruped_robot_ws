#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
 
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "IMUSensor.h"
#include "Actuators.h"


#define LED_PIN 13
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

bool newData = false;
bool recvComplete = false;
bool CAL = false;
bool secuence = false;
bool CTRL1 = false;
bool CTRL2 = false;
bool REST = false;
bool SAFE = true;
bool KILL = false;
bool RUN = false;
bool Push = false;
bool oPush = false;

struct LegsAngle anglesIK;

int PAGE = 0;
int maxPages = 4; // MAX PAGES
int pinVin = 27; // pin (teensy 4.1: 21)
float Vin = 0;
int pinLed = 31; // pin status led
int pinPush = 32; // pin push botton
float pressAt;
float fe1 = 0;
float fe2 = 0;
float fe3 = 0;
float fe4 = 0;
float publisher_latency = 0;


rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

rclc_executor_t executor_pub;
rcl_timer_t timer;
rcl_publisher_t imu_publisher;
std_msgs__msg__Float32MultiArray imu_msg;
rcl_publisher_t status_publisher;
std_msgs__msg__Int32MultiArray status_msg;


rclc_executor_t executor_sub;
rcl_subscription_t angles_subscriber;
std_msgs__msg__Float32MultiArray angles_msg;
bool micro_ros_init_successful;
long last_time;
IMUSensor imuSensor;
Actuators actuators;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    
    imuSensor.ReadData();
    publisher_latency = (micros() - last_time)/1000.0;
    last_time = micros();

    imu_msg.data.data[0] = publisher_latency;
    imu_msg.data.data[1] = imuSensor.GetPitch();
    imu_msg.data.data[2] = imuSensor.GetRoll();
    imu_msg.data.data[3] = imuSensor.GetYaw();
    rcl_publish(&imu_publisher, &imu_msg, NULL);

    status_msg.data.data[0] = Push;
    status_msg.data.data[1] = SAFE;
    status_msg.data.data[2] = REST;
    status_msg.data.data[3] = RUN;
    rcl_publish(&status_publisher, &status_msg, NULL);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * angles_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (angles_msg != NULL)
  {
    anglesIK.FR.tetta = angles_msg->data.data[0];
    anglesIK.FR.alpha = angles_msg->data.data[1];
    anglesIK.FR.gamma = angles_msg->data.data[2];
    anglesIK.FL.tetta = angles_msg->data.data[3];
    anglesIK.FL.alpha = angles_msg->data.data[4];
    anglesIK.FL.gamma = angles_msg->data.data[5];
    anglesIK.BR.tetta = angles_msg->data.data[6];
    anglesIK.BR.alpha = angles_msg->data.data[7];
    anglesIK.BR.gamma = angles_msg->data.data[8];
    anglesIK.BL.tetta = angles_msg->data.data[9];
    anglesIK.BL.alpha = angles_msg->data.data[10];
    anglesIK.BL.gamma = angles_msg->data.data[11];
    SAFE = actuators.StepMotors(RUN, SAFE, anglesIK);
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "hardware_manager", "", &support));
  
  // create angles subscriber
  RCCHECK(rclc_subscription_init_default(
    &angles_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "target_angle_msg"));

  // create imu publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "imu_sensor_msg"));
  imu_msg.data.capacity = 10;
  imu_msg.data.size = 4;
  imu_msg.data.data = (float*) malloc(3 * sizeof(float));
  
  // create status publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "robot_status_msg"));
  status_msg.data.capacity = 10;
  status_msg.data.size = 4;
  status_msg.data.data = (int32_t*) malloc(4 * sizeof(int32_t)); 

  // create timer,
  last_time = 0;
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executors
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  std_msgs__msg__Float32MultiArray__init(&angles_msg);
  angles_msg.data.capacity = 12;
  angles_msg.data.size = 12;
  angles_msg.data.data = (float*) malloc(angles_msg.data.capacity * sizeof(float));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &angles_subscriber, &angles_msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  free(angles_msg.data.data);
  free(status_msg.data.data);
  free(imu_msg.data.data);

  rcl_publisher_fini(&status_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_timer_fini(&timer);
  rcl_subscription_fini(&angles_subscriber, &node);
  rclc_executor_fini(&executor_pub);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  actuators.ConnectServos();
  imuSensor.Initialize();
  
  setupDisplay();
  startDisplay(PAGE);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      last_time = micros();
      RUN = true;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      //RUN = false;
      //destroy_entities();
      //state = WAITING_AGENT;
      WRITE_RESTART(0x5FA0004);
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
  
  //READ BATTERY STATUS
  Vin = float(analogRead(pinVin));
  //feed battery IN with two reference voltage to relate with analog signal
  Vin = map(Vin, 218, 83, 7.92 , 5.12);
  //Vin = 5.6 + 1.4*(1 + sin(3.14*t/5));
  //Vin = 7;

  //UPDATE BUTTON STATUS LED
  if (RUN == true) {
    digitalWrite(pinLed, HIGH);
  }
  else {
    digitalWrite(pinLed, LOW);
  }
  //SELECT SCREEN MODE AND UPDATE SCREEN
  selectDisplayPage();
  printDisplayLCD();

  if (KILL == true) {
    WRITE_RESTART(0x5FA0004);
  }
}