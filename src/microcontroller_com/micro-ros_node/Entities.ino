void InitTargetAngleMsg()
{
  sensor_msgs__msg__JointState__init(&angles_msg);
  angles_msg.name.size = numJoint;
  angles_msg.name.capacity = numJoint;
  angles_msg.name.data = (rosidl_runtime_c__String *)malloc(numJoint * sizeof(rosidl_runtime_c__String));
  for (int i = 0; i < numJoint; i++) {
    angles_msg.name.data[i].data = (char *)malloc(strlen(jointNames[i]) + 1);
    angles_msg.name.data[i].capacity = strlen(jointNames[i]) + 1;
    strcpy(angles_msg.name.data[i].data, jointNames[i]);
    angles_msg.name.data[i].size = strlen(jointNames[i]);
  }
  angles_msg.position.size = numJoint;
  angles_msg.position.capacity = numJoint;
  angles_msg.position.data = (double *)malloc(numJoint * sizeof(double));
}

void InitCalibrationMsg()
{
  std_msgs__msg__Int8MultiArray__init(&calibration_msg);
  calibration_msg.data.capacity = 10;
  calibration_msg.data.size = 2;
  calibration_msg.data.data = (int8_t*) malloc(status_msg.data.size * sizeof(int8_t)); 
}

void InitEncodersAngleMsg()
{
  sensor_msgs__msg__JointState__init(&encoders_msg);
  encoders_msg.name.size = numJoint;
  encoders_msg.name.capacity = numJoint;
  encoders_msg.name.data = (rosidl_runtime_c__String *)malloc(numJoint * sizeof(rosidl_runtime_c__String));
  for (int i = 0; i < numJoint; i++) {
    
  encoders_msg.name.data[i].data = (char *)malloc(strlen(jointNames[i]) + 1);
    
  encoders_msg.name.data[i].capacity = strlen(jointNames[i]) + 1;
    strcpy(
      encoders_msg.name.data[i].data, jointNames[i]);
    
  encoders_msg.name.data[i].size = strlen(jointNames[i]);
  }
  encoders_msg.position.size = numJoint;
  encoders_msg.position.capacity = numJoint;
  encoders_msg.position.data = (double *)malloc(numJoint * sizeof(double));
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
  InitTargetAngleMsg();

  // create calibration subscriber
  RCCHECK(rclc_subscription_init_default(
    &calibration_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
    "calibration_controller"));
  InitCalibrationMsg();


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
  InitEncodersAngleMsg();

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

  // create debugger publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug_msg"));
  std_msgs__msg__String__init(&debug_msg);
  const unsigned int DEBUG_MSG_CAPACITY = 30;
  debug_msg.data.data = malloc(DEBUG_MSG_CAPACITY);
  debug_msg.data.capacity = DEBUG_MSG_CAPACITY;
  debug_msg.data.size = strlen(debug_msg.data.data);

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executors
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &angles_subscriber, &angles_msg, &angles_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &calibration_subscriber, &calibration_msg, &calibration_callback, ON_NEW_DATA));

  return true;
}

void CleanupEncodersMsg()
{
  for (size_t i = 0; i < encoders_msg.name.size; i++) {
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
  //free(angles_msg.position.data);
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



