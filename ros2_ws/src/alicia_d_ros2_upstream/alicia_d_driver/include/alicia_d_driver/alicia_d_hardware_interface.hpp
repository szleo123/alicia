#ifndef ALICIA_D_DRIVER__ALICIA_D_HARDWARE_INTERFACE_HPP_
#define ALICIA_D_DRIVER__ALICIA_D_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <array>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "serial_communicator.hpp"
#include "alicia_d_data_parser_control.hpp"

using hardware_interface::return_type;

namespace alicia_d_driver
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC AliciaDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AliciaDHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication and data parsing
  std::unique_ptr<SerialCommunicator> communicator_;
  std::unique_ptr<AliciaDDataParserControl> data_parser_control_;
  std::string port_;
  bool debug_mode_;
  std::string gripper_type_param_;  // Gripper type parameter ("auto", "50mm", or "100mm")
  
  // Robot state
  std::vector<double> hw_positions_state_;
  std::vector<double> hw_positions_command_;
  std::vector<double> hw_velocities_state_;
  std::vector<double> hw_velocities_command_;
  
  // Thread safety
  std::mutex data_mutex_;
  
  // Speed control
  double default_speed_deg_s_;  // Default speed in deg/s (default: 20.0 deg/s)
  std::array<double, 6> joint_position_offsets_rad_{};
  
  // Command rate limiting (disabled for real-time control)
  rclcpp::Time last_write_time_;
  double min_write_period_;  // No rate limiting for real-time control
  
  // Hardware connection status
  bool hardware_connected_;  // Track if hardware is actually connected

  // Synchronization state
  bool has_received_joint_state_;
  bool commands_initialized_;
  bool has_sent_command_;
  std::vector<double> last_sent_positions_command_;
  std::vector<double> last_sent_velocities_command_;

  // Demonstration / hand-guiding mode bridge for the ros2_control path.
  std::shared_ptr<rclcpp::Node> command_bridge_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> command_bridge_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr demonstration_sub_;
  std::thread command_bridge_thread_;
  bool demonstration_mode_enabled_;

  void start_command_bridge();
  void stop_command_bridge();
  void demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);
  bool parse_joint_position_offsets(const std::string & raw_offsets);
};

}  // namespace alicia_d_driver

#endif  // ALICIA_D_DRIVER__ALICIA_D_HARDWARE_INTERFACE_HPP_
