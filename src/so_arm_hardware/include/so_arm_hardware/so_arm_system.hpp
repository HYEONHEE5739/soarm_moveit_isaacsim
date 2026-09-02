#ifndef SO_ARM_HARDWARE__SO_ARM_SYSTEM_HPP_
#define SO_ARM_HARDWARE__SO_ARM_SYSTEM_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "SMS_STS.h"

namespace so_arm_hardware
{

struct JointCalibration
{
  int motor_id;
  double center_tick; 
  double sign;
  int min_tick;
  int max_tick;
};

class SoArmSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SoArmSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> prev_positions_;

  std::string port_;
  int baudrate_{1000000};
  size_t write_log_counter_{0};

  SMS_STS servo_bus_;
  std::unordered_map<std::string, JointCalibration> calib_;

  void init_calibration();
  int joint_rad_to_tick(const std::string & joint_name, double rad) const;
  double tick_to_joint_rad(const std::string & joint_name, int tick) const;
};

}  // namespace so_arm_hardware

#endif  // SO_ARM_HARDWARE__SO_ARM_SYSTEM_HPP_