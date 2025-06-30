#include <functional>
#include <memory>
#include <numbers>

#include <mirte_telemetrix_cpp/modules/mpu9250_module.hpp>

using namespace std::placeholders;

MPU9250_sensor::MPU9250_sensor(NodeData node_data, MPU9250Data imu_data,
                               std::shared_ptr<tmx_cpp::Sensors> sensors)
    : Mirte_module(node_data, {imu_data.scl, imu_data.sda},
                   (ModuleData)imu_data),
      data(imu_data) {
  tmx->setI2CPins(imu_data.sda, imu_data.scl, imu_data.port);

  this->mpu9250 = std::make_shared<tmx_cpp::MPU9250_module>(
      imu_data.port, imu_data.addr,
      std::bind(&MPU9250_sensor::data_callback, this, _1, _2, _3, _4));

  imu_pub = nh->create_publisher<sensor_msgs::msg::Imu>(
      "imu/" + this->name + "/data", rclcpp::SystemDefaultsQoS());

  imu_service = nh->create_service<mirte_msgs::srv::GetImu>(
      "imu/" + this->name + "/get_data",
      std::bind(&MPU9250_sensor::get_imu_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  // NOTE: There is some covariance between the axes, but this is often
  // considered negligible.
  //  Covariance based on datasheet:
  //  https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
  msg.linear_acceleration_covariance[0] =
      std::pow(60.0 * 9.81 / 1000, 2); // Var_x
  msg.linear_acceleration_covariance[4] =
      std::pow(60.0 * 9.81 / 1000, 2); // Var_y
  msg.linear_acceleration_covariance[8] =
      std::pow(80.0 * 9.81 / 1000, 2); // Var_z

  msg.angular_velocity_covariance[0] =
      std::pow(5.0 * std::numbers::pi / 180.0, 2); // Var_x
  msg.angular_velocity_covariance[4] =
      std::pow(5.0 * std::numbers::pi / 180.0, 2); // Var_y
  msg.angular_velocity_covariance[8] =
      std::pow(5.0 * std::numbers::pi / 180.0, 2); // Var_z

  sensors->add_sens(this->mpu9250);
}

void MPU9250_sensor::update() {
  if (msg_mutex.try_lock()) {
    const std::lock_guard lock{msg_mutex, std::adopt_lock};
    msg.header = get_header();
    imu_pub->publish(msg);
  }
}

void MPU9250_sensor::data_callback(std::array<float, 3> acceleration,
                                   std::array<float, 3> gyro,
                                   std::array<float, 3> magnetic_field,
                                   std::array<float, 4> quaternion) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  msg.header = get_header();

  msg.linear_acceleration.x = acceleration[0] * 9.81;
  msg.linear_acceleration.y = acceleration[1] * 9.81;
  msg.linear_acceleration.z = acceleration[2] * 9.81;

  msg.angular_velocity.x = gyro[0] * std::numbers::pi / 180.0;
  msg.angular_velocity.y = gyro[1] * std::numbers::pi / 180.0;
  msg.angular_velocity.z = gyro[2] * std::numbers::pi / 180.0;

  msg.orientation.x = quaternion[0];
  msg.orientation.y = quaternion[1];
  msg.orientation.z = quaternion[2];
  msg.orientation.w = quaternion[3];

  imu_pub->publish(msg);
  device_timer->reset();
}

void MPU9250_sensor::get_imu_service_callback(
    const mirte_msgs::srv::GetImu::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetImu::Response::SharedPtr res) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  res->data = sensor_msgs::msg::Imu(msg);
}

std::vector<std::shared_ptr<MPU9250_sensor>>
MPU9250_sensor::get_mpu_modules(NodeData node_data,
                                std::shared_ptr<Parser> parser,
                                std::shared_ptr<tmx_cpp::Sensors> sensors) {
  std::vector<std::shared_ptr<MPU9250_sensor>> mpu_modules;
  auto mpu_data = parse_all_modules<MPU9250Data>(parser, node_data.board);
  for (auto mpu : mpu_data) {
    auto mpu_module = std::make_shared<MPU9250_sensor>(node_data, mpu, sensors);
    mpu_modules.push_back(mpu_module);
  }
  return mpu_modules;
}
