#include "alicia_d_driver/alicia_d_hardware_interface.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

namespace alicia_d_driver
{

bool AliciaDHardwareInterface::parse_joint_position_offsets(const std::string & raw_offsets)
{
  joint_position_offsets_rad_.fill(0.0);
  if (raw_offsets.empty()) {
    return true;
  }

  std::string normalized = raw_offsets;
  for (char & ch : normalized) {
    if (ch == ';') {
      ch = ',';
    }
  }

  std::stringstream ss(normalized);
  std::string item;
  size_t index = 0;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) {
      continue;
    }
    if (index >= joint_position_offsets_rad_.size()) {
      return false;
    }
    joint_position_offsets_rad_[index++] = std::stod(item);
  }

  return index == joint_position_offsets_rad_.size();
}

CallbackReturn AliciaDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  port_ = info_.hardware_parameters["port"];
  debug_mode_ = info_.hardware_parameters.count("debug_mode") ? 
                (info_.hardware_parameters["debug_mode"] == "true") : false;
  
  // Gripper type (required: "50mm" or "100mm", default "50mm")
  gripper_type_param_ = info_.hardware_parameters.count("gripper_type") ? 
                         info_.hardware_parameters["gripper_type"] : "50mm";
  
  // Speed control parameter (default 20.0 deg/s)
  default_speed_deg_s_ = info_.hardware_parameters.count("default_speed_deg_s") ? 
                         std::stod(info_.hardware_parameters["default_speed_deg_s"]) : 20.0;
  const std::string raw_joint_offsets = info_.hardware_parameters.count("joint_position_offsets_rad") ?
                                        info_.hardware_parameters["joint_position_offsets_rad"] : "0,0,0,0,0,0";
  if (!parse_joint_position_offsets(raw_joint_offsets))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("AliciaDHardwareInterface"),
      "Invalid joint_position_offsets_rad parameter: '%s'",
      raw_joint_offsets.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Default speed configured: %.1f deg/s", default_speed_deg_s_);
  RCLCPP_INFO(
    rclcpp::get_logger("AliciaDHardwareInterface"),
    "Joint position offsets (rad): [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]",
    joint_position_offsets_rad_[0], joint_position_offsets_rad_[1], joint_position_offsets_rad_[2],
    joint_position_offsets_rad_[3], joint_position_offsets_rad_[4], joint_position_offsets_rad_[5]);

  // Initialize state and command vectors
  hw_positions_state_.resize(info_.joints.size(), 0.0);
  hw_positions_command_.resize(info_.joints.size(), 0.0);
  hw_velocities_state_.resize(info_.joints.size(), 0.0);
  hw_velocities_command_.resize(info_.joints.size(), 0.0);
  last_sent_positions_command_.resize(info_.joints.size(), 0.0);
  last_sent_velocities_command_.resize(info_.joints.size(), 0.0);

  last_write_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  min_write_period_ = 0.0;  // No rate limiting - send commands every cycle for real-time control

  // Initialize hardware connection status
  hardware_connected_ = false;
  has_received_joint_state_ = false;
  commands_initialized_ = false;
  has_sent_command_ = false;
  demonstration_mode_enabled_ = false;

  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Initialized hardware interface (using unified data parser control)");
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
              "Port: %s, Real-time control enabled", 
              port_.empty() ? "(auto-detect)" : port_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn AliciaDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Configuring hardware interface...");
  
  // Create serial communicator and data parser control (matching driver node)
  communicator_ = std::make_unique<SerialCommunicator>(port_, debug_mode_);
  data_parser_control_ = std::make_unique<AliciaDDataParserControl>(
      communicator_.get(), rclcpp::get_logger("AliciaDHardwareInterface"), debug_mode_, gripper_type_param_);
  start_command_bridge();
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AliciaDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_state_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AliciaDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_command_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_command_[i]));
  }

  return command_interfaces;
}

CallbackReturn AliciaDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Activating hardware interface...");

  if (!command_bridge_node_)
  {
    start_command_bridge();
  }
  
  // Try to connect to serial port
  if (communicator_->connect())
  {
    hardware_connected_ = true;
    has_received_joint_state_ = false;
    commands_initialized_ = false;
    has_sent_command_ = false;
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "Connected to robot. Starting parsing thread and querying information.");

    // Start parsing thread (matching driver node - background thread for continuous data parsing)
    data_parser_control_->start_parsing_thread();

    // Query all information types (matching driver node)
    data_parser_control_->acquire_info("version", true, 3.0, 0.2);
    data_parser_control_->acquire_info("temperature", true, 2.0, 0.2);
    data_parser_control_->acquire_info("velocity", true, 2.0, 0.2);
    data_parser_control_->acquire_info("self_check", true, 2.0, 0.2);
    
    // Print all available information
    data_parser_control_->print_information();

    // Enable torque using data parser control unless hand-guiding mode has
    // already been requested through the bridge topic.
    data_parser_control_->torque_control(demonstration_mode_enabled_ ? "off" : "on");
  }
  else
  {
    hardware_connected_ = false;
    has_received_joint_state_ = false;
    commands_initialized_ = false;
    has_sent_command_ = false;
    RCLCPP_WARN(rclcpp::get_logger("AliciaDHardwareInterface"), 
                "No hardware connected. Running in simulation/demo mode. "
                "Hardware interface will accept commands but they will not be sent to robot.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn AliciaDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"), "Deactivating hardware interface...");
  
  // Stop parsing thread (matching driver node) only if hardware was connected
  if (hardware_connected_ && data_parser_control_)
  {
    data_parser_control_->stop_parsing_thread();
  }
  
  // Disconnect from serial port only if hardware was connected
  if (hardware_connected_ && communicator_)
  {
    communicator_->disconnect();
  }

  stop_command_bridge();
  
  hardware_connected_ = false;
  has_received_joint_state_ = false;
  commands_initialized_ = false;
  has_sent_command_ = false;

  return CallbackReturn::SUCCESS;
}

void AliciaDHardwareInterface::start_command_bridge()
{
  stop_command_bridge();

  command_bridge_node_ = std::make_shared<rclcpp::Node>("alicia_d_hardware_command_bridge");
  demonstration_sub_ = command_bridge_node_->create_subscription<std_msgs::msg::Bool>(
    "/demonstration",
    10,
    std::bind(&AliciaDHardwareInterface::demonstration_mode_callback, this, std::placeholders::_1));

  command_bridge_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  command_bridge_executor_->add_node(command_bridge_node_);
  command_bridge_thread_ = std::thread([this]() {
    command_bridge_executor_->spin();
  });

  RCLCPP_INFO(
    rclcpp::get_logger("AliciaDHardwareInterface"),
    "Started /demonstration command bridge for real-robot hand-guiding mode.");
}

void AliciaDHardwareInterface::stop_command_bridge()
{
  if (command_bridge_executor_)
  {
    command_bridge_executor_->cancel();
  }

  if (command_bridge_thread_.joinable())
  {
    command_bridge_thread_.join();
  }

  if (command_bridge_executor_ && command_bridge_node_)
  {
    command_bridge_executor_->remove_node(command_bridge_node_);
  }

  demonstration_sub_.reset();
  command_bridge_executor_.reset();
  command_bridge_node_.reset();
}

void AliciaDHardwareInterface::demonstration_mode_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  demonstration_mode_enabled_ = msg->data;

  if (!hardware_connected_ || !data_parser_control_)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("AliciaDHardwareInterface"),
      "Received /demonstration=%s but hardware is not connected yet.",
      demonstration_mode_enabled_ ? "true" : "false");
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  const char * torque_command = demonstration_mode_enabled_ ? "off" : "on";
  const bool ok = data_parser_control_->torque_control(torque_command);
  if (ok)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("AliciaDHardwareInterface"),
      "%s hand-guiding mode via /demonstration.",
      demonstration_mode_enabled_ ? "Enabled" : "Disabled");
  }
  else
  {
    RCLCPP_WARN(
      rclcpp::get_logger("AliciaDHardwareInterface"),
      "Failed to change torque state for /demonstration=%s.",
      demonstration_mode_enabled_ ? "true" : "false");
  }
}

return_type AliciaDHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If hardware is not connected, simulate state updates (copy commands to state for simulation)
  if (!hardware_connected_ || !communicator_ || !communicator_->is_connected() || !data_parser_control_)
  {
    // In simulation mode, update state to match commands (simulate ideal robot)
    std::lock_guard<std::mutex> lock(data_mutex_);
    hw_positions_state_ = hw_positions_command_;
    // Set velocities to zero in simulation (or copy from commands if provided)
    for (size_t i = 0; i < hw_velocities_state_.size(); ++i)
    {
      if (i < hw_velocities_command_.size())
      {
        hw_velocities_state_[i] = hw_velocities_command_[i];
      }
      else
      {
        hw_velocities_state_[i] = 0.0;
      }
    }
    return return_type::OK;
  }

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  static rclcpp::Time last_joint_request(0, 0, RCL_STEADY_TIME);
  rclcpp::Time now = steady_clock.now();
  if ((now - last_joint_request).seconds() >= 0.01)  // Request at 100 Hz to reduce start-state drift
  {
    data_parser_control_->acquire_info("joint", false);
    last_joint_request = now;
  }
  
  // Update state from parser (data is parsed by background thread)
  auto joint_state = data_parser_control_->get_joint_state();
  auto velocity_data = data_parser_control_->get_velocity_data();
  
  if (joint_state.has_value())
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Update joint positions (first 6 joints)
    for (size_t i = 0; i < 6 && i < joint_state->angles.size() && i < hw_positions_state_.size(); ++i)
    {
      hw_positions_state_[i] = joint_state->angles[i] + joint_position_offsets_rad_[i];
    }
    
    // Update gripper position (convert from 0-1000 to meters)
    if (hw_positions_state_.size() > 6)
    {
      hw_positions_state_[6] = data_parser_control_->gripper_value_to_position(joint_state->gripper);
    }
    
    // Update velocities if available
    if (velocity_data.has_value() && velocity_data->velocities.size() >= 6)
    {
      // Convert from deg/s to rad/s for first 6 joints
      for (size_t i = 0; i < 6 && i < velocity_data->velocities.size() && i < hw_velocities_state_.size(); ++i)
      {
        hw_velocities_state_[i] = velocity_data->velocities[i] * M_PI / 180.0;
      }
      // Gripper velocity (if available, otherwise 0)
      if (hw_velocities_state_.size() > 6)
      {
        hw_velocities_state_[6] = 0.0;  // Gripper velocity not typically reported
      }
    }

    has_received_joint_state_ = true;

    // Synchronize command buffers to the first measured robot state before
    // allowing any hardware writes. This avoids sending an all-zero command
    // on activation before the interface has observed the real arm pose.
    if (!commands_initialized_)
    {
      hw_positions_command_ = hw_positions_state_;
      std::fill(hw_velocities_command_.begin(), hw_velocities_command_.end(), 0.0);
      last_sent_positions_command_ = hw_positions_command_;
      last_sent_velocities_command_ = hw_velocities_command_;
      commands_initialized_ = true;
      RCLCPP_INFO(
        rclcpp::get_logger("AliciaDHardwareInterface"),
        "Synchronized command interfaces to measured robot state. Hardware writes enabled."
      );
    }
  }

  return return_type::OK;
}

return_type AliciaDHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If hardware is not connected, simulate command acceptance (state will be updated in read())
  if (!hardware_connected_ || !communicator_ || !communicator_->is_connected() || !data_parser_control_)
  {
    // In simulation mode, commands are accepted but not sent to hardware
    // The read() method will copy commands to state to simulate movement
    return return_type::OK;
  }

  if (!has_received_joint_state_ || !commands_initialized_)
  {
    return return_type::OK;
  }

  // Real-time control: send commands every cycle (no rate limiting)
  // The robot hardware can handle high-frequency commands for smooth real-time control
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  constexpr double POSITION_EPS = 1e-3;
  constexpr double VELOCITY_EPS = 1e-4;
  bool command_changed = !has_sent_command_;
  if (!command_changed)
  {
    for (size_t i = 0; i < hw_positions_command_.size(); ++i)
    {
      if (std::abs(hw_positions_command_[i] - last_sent_positions_command_[i]) > POSITION_EPS)
      {
        command_changed = true;
        break;
      }
      if (i < hw_velocities_command_.size() &&
          std::abs(hw_velocities_command_[i] - last_sent_velocities_command_[i]) > VELOCITY_EPS)
      {
        command_changed = true;
        break;
      }
    }
  }

  // Avoid nudging the robot on startup by staying passive until the controller
  // actually changes the commanded target.
  if (!command_changed)
  {
    return return_type::OK;
  }

  if (!has_sent_command_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("AliciaDHardwareInterface"),
      "Detected first controller target change. Sending hardware commands at %.1f deg/s",
      default_speed_deg_s_);
  }
  
  // Extract joint angles (first 6 joints)
  std::vector<double> joint_angles;
  for (size_t i = 0; i < 6 && i < hw_positions_command_.size(); ++i)
  {
    joint_angles.push_back(hw_positions_command_[i] - joint_position_offsets_rad_[i]);
  }
  
  // Extract gripper position and convert to value (0-1000)
  double gripper_value = -1.0;
  if (hw_positions_command_.size() > 6)
  {
    gripper_value = data_parser_control_->gripper_position_to_value(hw_positions_command_[6]);
  }
  
  double speed_deg_s = default_speed_deg_s_;
  
  // Periodic logging to verify speed is being used (log every 2000 calls = ~10 seconds at 200Hz)
  // or when speed changes significantly
  static int write_count = 0;
  static double last_logged_speed = -1.0;
  write_count++;
  if (write_count % 2000 == 0 || std::abs(speed_deg_s - last_logged_speed) > 1.0)
  {
    RCLCPP_INFO(rclcpp::get_logger("AliciaDHardwareInterface"),
                "Sending joint command with speed: %.1f deg/s", speed_deg_s);
    last_logged_speed = speed_deg_s;
  }
  
  data_parser_control_->set_joint_and_gripper(joint_angles, gripper_value, speed_deg_s);
  last_sent_positions_command_ = hw_positions_command_;
  last_sent_velocities_command_ = hw_velocities_command_;
  has_sent_command_ = true;

  return return_type::OK;
}


}  // namespace alicia_d_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(alicia_d_driver::AliciaDHardwareInterface, hardware_interface::SystemInterface)
