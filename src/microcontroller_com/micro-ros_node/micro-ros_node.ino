#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
 
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/header.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8_multi_array.h>

#include "IMUSensor.h"
#include "Actuators.h"
#include "ActuatorsEncoders.h"



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

bool CTRL1 = false;
bool CTRL2 = false;
bool REST = false;
bool SAFE = true;
bool KILL = false;
bool isRunning = false;
bool RUN = false;
bool Push = false;
bool oPush = false;
int actualState = 0;
int pinVin = 27; // pin (teensy 4.1: 21)
float Vin = 0;
int pinLed = 31; // pin status led
int pinPush = 32; // pin push botton
float pressAt;
float publisher_latency = 0;

struct LegsAngle anglesIK;

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

rclc_executor_t executor_pub;
rcl_timer_t timer;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t encoders_publisher;
sensor_msgs__msg__JointState encoders_msg;
rcl_publisher_t battery_state_publisher;
sensor_msgs__msg__BatteryState battery_msg;
rcl_publisher_t status_publisher;
std_msgs__msg__Int8MultiArray status_msg;

rclc_executor_t executor_sub;
rcl_subscription_t angles_subscriber;
sensor_msgs__msg__JointState angles_msg;


bool micro_ros_init_successful;
const unsigned int timer_timeout = 3;
long last_time;
IMUSensor imuSensor;
Actuators actuators;
ActuatorsEncoders actuatorsEncoders;
const int numJoint = 12;
const char *jointNames[numJoint] = {"coxaF_FR","femurF_FR","tibiaF_FR",
                                    "coxaF_FL","femurF_FL","tibiaF_FL",
                                    "coxaF_BR","femurF_BR","tibiaF_BR",
                                    "coxaF_BL","femurF_BL","tibiaF_BL"};

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void PublishIMUSensorData()
{
  imuSensor.ReadData();
  imu_msg.orientation.x = imuSensor.GetPitch();
  imu_msg.orientation.y = imuSensor.GetRoll();
  imu_msg.orientation.z = imuSensor.GetYaw();
  imu_msg.linear_acceleration.x = imuSensor.GetXacc();
  imu_msg.linear_acceleration.y = imuSensor.GetXacc();
  imu_msg.linear_acceleration.z = imuSensor.GetXacc();
  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void PublishStatusData()
{
  status_msg.data.data[0] = KILL;
  status_msg.data.data[1] = SAFE;
  status_msg.data.data[2] = REST;
  status_msg.data.data[3] = RUN;
  status_msg.data.data[4] = actualState;
  status_msg.data.data[5] = publisher_latency;
  rcl_publish(&status_publisher, &status_msg, NULL);
}

void PublishEncodersData()
{
  actuatorsEncoders.ReadRawEncoders();
  for (int i = 0; i < numJoint; i++)
  {
    encoders_msg.position.data[i] = actuatorsEncoders.GetEncoderRawValue(i);
  }
  rcl_publish(&encoders_publisher, &encoders_msg, NULL);
}

void PublishBatteryStateData()
{
  battery_msg.voltage = Vin;
  rcl_publish(&battery_state_publisher, &battery_msg, NULL);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    publisher_latency = (micros() - last_time)/1000.0;
    last_time = micros();
    
    PublishIMUSensorData();
    PublishStatusData();
    PublishEncodersData();
    PublishBatteryStateData();
  }
}

void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__JointState * angles_msg = (const sensor_msgs__msg__JointState *)msgin;
  if (angles_msg != NULL)
  {
    if (!isRunning)
    {
      RUN = true;
      isRunning = true;
    }
    
    for (int i = 0; i < numJoint; i++)
    {
      anglesIK.asArray[i] = angles_msg->position.data[i];
      anglesIK.asArray[i] = rad2deg(anglesIK.asArray[i]);
    }
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "target_angle_msg"));
  sensor_msgs__msg__JointState__init(&angles_msg);
  angles_msg.position.size = numJoint;
  angles_msg.position.capacity = numJoint;
  angles_msg.position.data = (double *)malloc(numJoint * sizeof(double));

  // create imu publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_sensor_msg"));

  // create encoders publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &encoders_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "encoders_sensor_msg"));
  sensor_msgs__msg__JointState__init(&encoders_msg);
  encoders_msg.name.size = numJoint;
  encoders_msg.name.capacity = numJoint;
  encoders_msg.name.data = (rosidl_runtime_c__String *)malloc(numJoint * sizeof(rosidl_runtime_c__String));
  for (int i = 0; i < numJoint; i++) {
    encoders_msg.name.data[i].data = (char *)malloc(strlen(jointNames[i]) + 1);
    encoders_msg.name.data[i].capacity = strlen(jointNames[i]) + 1;
    strcpy(encoders_msg.name.data[i].data, jointNames[i]);
    encoders_msg.name.data[i].size = strlen(jointNames[i]);
  }
  encoders_msg.position.size = numJoint;
  encoders_msg.position.capacity = numJoint;
  encoders_msg.position.data = (double *)malloc(numJoint * sizeof(double));

  // create battery status publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &battery_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "battery_state_msg"));
  // create status publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
    "robot_status_msg"));
  status_msg.data.capacity = 1;
  status_msg.data.size = 6;
  status_msg.data.data = (int8_t*) malloc(status_msg.data.size * sizeof(int8_t)); 

  // create timer,
  last_time = 0;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executors
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &angles_subscriber, &angles_msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

void CleanupEncodersMsg()
{
  for (int i = 0; i < encoders_msg.name.size; i++) {
    free(encoders_msg.name.data[i].data); // Liberar memoria de cada cadena
  }
  free(encoders_msg.name.data);           // Liberar la secuencia de nombres
  free(encoders_msg.position.data);       // Liberar la secuencia de posiciones
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  free(status_msg.data.data);
  free(angles_msg.position.data);
  CleanupEncodersMsg();

  rcl_publisher_fini(&status_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&encoders_publisher, &node);
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
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(timer_timeout));
        rclc_executor_spin_some(&executor_sub, 100);
      }
      break;
    case AGENT_DISCONNECTED:
      //TODO: make reconnection works, seems there is troubles destroying entities. 
      //destroy_entities();
      //state = WAITING_AGENT;
      RUN = false;
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
  Vin = map(Vin, 218, 83, 7.92 , 5.12); // In mV
  //Vin = 5.6 + 1.4*(1 + sin(3.14*t/5));
  //Vin = 7;

  //UPDATE BUTTON STATUS LED
  if (RUN == true) {
    digitalWrite(pinLed, HIGH);
  }
  else {
    digitalWrite(pinLed, LOW);
  }
  if (KILL == true) {
    WRITE_RESTART(0x5FA0004);
  }
}