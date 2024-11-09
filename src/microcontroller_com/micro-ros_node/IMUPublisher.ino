/*z#include <std_msgs/msg/float32_multi_array.h>



rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;


void Initialize()
{
  imuSensor.Initialize();
}

void CreatePublisher(rcl_node_t node)
{
// create imu publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "imu_angles_Float32MultiArray"));

  msg.data.capacity = 10;
  msg.data.size = 4;
  msg.data.data = (float*) malloc(3 * sizeof(float));

  last_time = 0;
}

void PublishData()
{
    // set topic values and publish
    imuSensor.ReadData();
    publisher_latency = (micros() - last_time)/1000.0;
    last_time = micros();

    msg.data.data[0] = publisher_latency;
    msg.data.data[1] = imuSensor.GetPitch();
    msg.data.data[2] = imuSensor.GetRoll();
    msg.data.data[3] = imuSensor.GetYaw();
    rcl_publish(&publisher, &msg, NULL);
}

void DestroyPub(rcl_node_t node)
{
    free(msg.data.data);
    rcl_publisher_fini(&publisher, &node);
}*/