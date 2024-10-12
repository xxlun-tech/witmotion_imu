/**
 * @file hwt905_node.cpp
 * @author ponomarevda96@gmail.com
 * @author xxlshi
 */

#include "hwt905_driver.hpp"
#include "serial_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#define DEG_TO_RAD  (0.01745329)

class Hwt905RosDriver : public rclcpp::Node
{
public:
  Hwt905RosDriver();

  virtual ~Hwt905RosDriver() = default;

private:
  void timer_callback();

  void read_serial_data();

  void publish();

  void process_parsed_result(const Hwt905_DataType_t & data_type);

  Hwt905Driver _hwt905_driver;

  SerialDriver _serial_driver;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

  sensor_msgs::msg::Imu _imu_msg;

  Hwt905_Time_t _time;

  Hwt905_Acceleration_t _accel;

  Hwt905_AngularVelocity_t _ang_vel;

  Hwt905_Angle_t _angle;

  Hwt905_Magnetic_t _mag;

  Hwt905_Quaternion_t _quaternion;

  rclcpp::TimerBase::SharedPtr _timer;  // Timer for periodic execution
};

Hwt905RosDriver::Hwt905RosDriver() : Node("hwt905_node")
{
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 460800);
  this->declare_parameter<std::string>("frame_id", "imu_link");
  this->declare_parameter<std::string>("imu_topic", "/imu");

  std::string serial_port;
  std::string imu_topic;
  std::string frame_id;
  int baud_rate;

  this->get_parameter("serial_port", serial_port);
  this->get_parameter("baud_rate", baud_rate);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("imu_topic", imu_topic);

  _serial_driver.init(serial_port, baud_rate);

  _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

  _imu_msg.header.stamp = this->now();
  _imu_msg.header.frame_id = frame_id;

  // Initial quaternion setup
  _quaternion.q_0 = 1.0;
  _quaternion.q_1 = 0.0;
  _quaternion.q_2 = 0.0;
  _quaternion.q_3 = 0.0;

  _timer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Hwt905RosDriver::timer_callback, this));
}

void Hwt905RosDriver::timer_callback()
{
  read_serial_data();  // Read data from the serial port
  publish();  // Publish the IMU data
}

void Hwt905RosDriver::publish()
{
  _imu_msg.header.stamp = this->now();

  _imu_msg.orientation.x = _quaternion.q_1;
  _imu_msg.orientation.y = _quaternion.q_2;
  _imu_msg.orientation.z = _quaternion.q_3;
  _imu_msg.orientation.w = _quaternion.q_0;

  _imu_msg.angular_velocity.x = _ang_vel.wx * DEG_TO_RAD;
  _imu_msg.angular_velocity.y = _ang_vel.wy * DEG_TO_RAD;
  _imu_msg.angular_velocity.z = _ang_vel.wz * DEG_TO_RAD;

  _imu_msg.linear_acceleration.x = _accel.ax;
  _imu_msg.linear_acceleration.y = _accel.ay;
  _imu_msg.linear_acceleration.z = _accel.az;

  _imu_pub->publish(_imu_msg);
}

void Hwt905RosDriver::read_serial_data()
{
  constexpr const size_t MAX_SERIAL_BUFFER_RECV_SIZE = 256;
  uint8_t serial_recv_buf[MAX_SERIAL_BUFFER_RECV_SIZE];

  int32_t num_of_recv_bytes = _serial_driver.spin(serial_recv_buf, MAX_SERIAL_BUFFER_RECV_SIZE);
  if (num_of_recv_bytes < 0) {
    RCLCPP_ERROR(this->get_logger(), "Error reading serial data");
    return;
  }

  for (int32_t byte_idx = 0; byte_idx < num_of_recv_bytes; ++byte_idx) {
    auto data_type = _hwt905_driver.process_next_byte(serial_recv_buf[byte_idx]);
    process_parsed_result(data_type);
  }
}

void Hwt905RosDriver::process_parsed_result(const Hwt905_DataType_t & data_type)
{
  switch (data_type) {
    case DATA_TYPE_NONE:
      break;
    case DATA_TYPE_TIME:
      _hwt905_driver.get_time(&_time);
      break;
    case DATA_TYPE_ACCEL:
      _hwt905_driver.get_acceleration(&_accel);
      break;
    case DATA_TYPE_ANG_VEL:
      _hwt905_driver.get_angular_velocity(&_ang_vel);
      break;
    case DATA_TYPE_ANGLE:
      _hwt905_driver.get_angle(&_angle);
      break;
    case DATA_TYPE_MAG:
      _hwt905_driver.get_magnetic_field(&_mag);
      break;
    case DATA_TYPE_QUATERNION:
      _hwt905_driver.get_quaternion(&_quaternion);
      break;
    default:
      break;
  }
}

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Hwt905RosDriver>());
  rclcpp::shutdown();

  return 0;
}
