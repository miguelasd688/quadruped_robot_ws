### Firmware for teensy 4.1 using arduino:

#### Connect agent to enable microros transports:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

#### MicroRosNode

Subscriptors:

* For servomotors angles, you can directly publish to debug motor positions.
  ```
  ros2 topic pub /target_angle_msg std_msgs/msg/Float32MultiArray data:\ [0,-45,90,0,-45,90,0,45,-90,0,45,-90]
  ```

Publishers:

* Imu sensor data: `ros2 topic echo /imu_sensor_msg`
* Robot status control flags,

#### Setup to compile and upload firmware:

1. Done with Ubuntu 22.04
2. Install Arduino IDE 2.2.1 (arduino-ide 2 for Raspberry Pi [https://github.com/koendv/arduino-ide-raspberrypi](https://github.com/koendv/arduino-ide-raspberrypi))

   I also tried to compile with a x64 pc with Arduino IDE 2.3.3.
3. Install Teensyduino 1.59.0 following the oficial documentation.
4. Patch micro-ros to work with Teensyduino at `~/.Arduino15/packages/teensy/jardware/avr/1.59.0/platform.txt`

   ```
   curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
   ```
5. micro_ros_arduino humble version [https://github.com/micro-ROS/micro_ros_arduino/tree/humble?tab=readme-ov-file](https://https://github.com/micro-ROS/micro_ros_arduino/tree/humble?tab=readme-ov-file). install it as .zip from Arduino IDE.
6. if you need to rebuild library just erase it from `~/Arduino/libraries` folder and reinstall from Arduino IDE
7. other librearies needed:

   * Adafruit BNO055 (for IMU)
   * Adafruit ST7735/ST7789 library (for LCD)
   * SimpleKalmanFilter
