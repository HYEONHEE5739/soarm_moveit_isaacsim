#include "so_arm_hardware/so_arm_system.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so_arm_hardware
{

namespace
{
constexpr double kTicksPerRad = 4096.0 / (2.0 * M_PI);

// 처음에는 너무 빠르지 않게 시작
constexpr int kDefaultSpeed = 800;
constexpr int kDefaultAcc   = 30;
}  // namespace

hardware_interface::CallbackReturn SoArmSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto num_joints = info_.joints.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);
  prev_positions_.resize(num_joints, 0.0);

  const auto port_it = info_.hardware_parameters.find("port");
  const auto baudrate_it = info_.hardware_parameters.find("baudrate");

  if (port_it == info_.hardware_parameters.end() ||
      baudrate_it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SoArmSystem"),
      "Missing required hardware parameters: port or baudrate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = port_it->second;

  try
  {
    baudrate_ = std::stoi(baudrate_it->second);
  }
  catch (const std::exception &)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SoArmSystem"),
      "Invalid baudrate: %s",
      baudrate_it->second.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  init_calibration();

  // URDF / controllers에 있는 joint 이름이 calibration map에 다 있는지 검사
  for (const auto & joint : info_.joints)
  {
    if (calib_.find(joint.name) == calib_.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SoArmSystem"),
        "No calibration entry for joint: %s",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SoArmSystem"),
    "Initialized SoArmSystem with %zu joints, port=%s, baudrate=%d",
    num_joints, port_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  // SDK가 포트 open + baudrate 설정을 담당
  if (!servo_bus_.begin(baudrate_, port_.c_str()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SoArmSystem"),
      "Failed to open servo bus on %s at %d baud",
      port_.c_str(), baudrate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SoArmSystem"),
    "Opened servo bus on %s at %d baud",
    port_.c_str(), baudrate_);
  
  RCLCPP_INFO(rclcpp::get_logger("SoArmSystem"), "=== ENTER INITMOTOR LOOP ===");
  
  for (const auto & [joint_name, c] : calib_)
  {
    const int ok = servo_bus_.InitMotor(c.motor_id, 0, 1);

    if (!ok)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("SoArmSystem"),
        "InitMotor failed for %s id=%d",
        joint_name.c_str(), c.motor_id);
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("SoArmSystem"),
        "InitMotor ok for %s id=%d",
        joint_name.c_str(), c.motor_id);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SoArmSystem"), "Configured SoArmSystem");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  // 시작 시 현재 실제 모터 위치를 읽어 command/state를 맞춤
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;
    const auto & c = calib_.at(joint_name);

    const int pos = servo_bus_.ReadPos(c.motor_id);
    if (pos < 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("SoArmSystem"),
        "ReadPos failed at activate for joint=%s id=%d, using 0 rad",
        joint_name.c_str(), c.motor_id);

      hw_positions_[i] = 0.0;
      hw_commands_[i] = 0.0;
      prev_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
      continue;
    }

    const double rad = tick_to_joint_rad(joint_name, pos);
    hw_positions_[i] = rad;
    hw_commands_[i] = rad;
    prev_positions_[i] = rad;
    hw_velocities_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SoArmSystem"), "Activated SoArmSystem");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // SDK에 명시적 close/end API가 있으면 여기서 호출
  // 예: servo_bus_.end();

  RCLCPP_INFO(rclcpp::get_logger("SoArmSystem"), "Deactivated SoArmSystem");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SoArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SoArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type SoArmSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;
    const auto & c = calib_.at(joint_name);

    const int pos = servo_bus_.ReadPos(c.motor_id);
    if (pos < 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("SoArmSystem"),
        "ReadPos failed for joint=%s id=%d",
        joint_name.c_str(), c.motor_id);
      continue;
    }

    const double rad = tick_to_joint_rad(joint_name, pos);
    hw_positions_[i] = rad;

    if (dt > 1e-9)
    {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / dt;
    }
    else
    {
      hw_velocities_[i] = 0.0;
    }

    prev_positions_[i] = hw_positions_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SoArmSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;
    const auto & c = calib_.at(joint_name);

    const int tick = joint_rad_to_tick(joint_name, hw_commands_[i]);

    const int ok = servo_bus_.WritePosEx(
      c.motor_id,
      tick,
      kDefaultSpeed,
      kDefaultAcc);

    if (!ok)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("SoArmSystem"),
        "WritePosEx failed for joint=%s id=%d tick=%d",
        joint_name.c_str(), c.motor_id, tick);
    }

    if (write_log_counter_ % 50 == 0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("SoArmSystem"),
        "joint=%s id=%d rad=%.4f tick=%d",
        joint_name.c_str(), c.motor_id, hw_commands_[i], tick);
    }
  }

  ++write_log_counter_;
  return hardware_interface::return_type::OK;
}

void SoArmSystem::init_calibration()
{
  calib_.clear();

  // 중요:
  // 아래 값들은 "초기값"이다.
  // 실제 SOARM follower에 맞게 center_tick / sign은 꼭 보정해야 한다.
  calib_["shoulder_pan"]  = {1, 2048.0, 1.0, 0, 4095};
  calib_["shoulder_lift"] = {2, 2048.0, 1.0, 0, 4095};
  calib_["elbow_flex"]    = {3, 2048.0, 1.0, 0, 4095};
  calib_["wrist_flex"]    = {4, 2048.0, 1.0, 0, 4095};
  calib_["wrist_roll"]    = {5, 2048.0, 1.0, 0, 4095};
  calib_["gripper"]       = {6, 2048.0, 1.0, 0, 4095};
}

int SoArmSystem::joint_rad_to_tick(const std::string & joint_name, double rad) const
{
  const auto & c = calib_.at(joint_name);

  const double raw_tick = c.center_tick + c.sign * rad  * kTicksPerRad;
  int tick = static_cast<int>(std::lround(raw_tick));
  tick = std::clamp(tick, c.min_tick, c.max_tick);

  return tick;
}

double SoArmSystem::tick_to_joint_rad(const std::string & joint_name, int tick) const
{
  const auto & c = calib_.at(joint_name);

  return (static_cast<double>(tick) - c.center_tick) / (c.sign * kTicksPerRad);
}

}  // namespace so_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  so_arm_hardware::SoArmSystem,
  hardware_interface::SystemInterface)