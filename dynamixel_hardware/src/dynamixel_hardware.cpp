// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include <chrono>
#include <ctime>
#include <fstream>


#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

struct MeasurementData {//structure to store measurements
    double relative_time_miliseconds;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
};

std::vector<MeasurementData> collected_measurements;//vector to store collected measurements
std::chrono::time_point<std::chrono::system_clock> start_time;//time point to store the start time of the first measurement

// Function to save collected measurements to a CSV file
void saveMeasurementsToFile(const std::string& filename) {
    // Pełna ścieżka do pliku
    std::string fullPath = "/home/jaku6m/Desktop/DynamixelData/" + filename;

    std::ofstream outputFile(fullPath);

    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << fullPath << std::endl;
        return;
    }

    for (const auto& measurement : collected_measurements) {
        for (size_t i = 0; i < measurement.positions.size(); ++i) {
            outputFile << std::fixed << std::setprecision(2) << measurement.relative_time_miliseconds << ",";
            outputFile << measurement.positions[i] << ",";
            outputFile << measurement.velocities[i] << ",";
            outputFile << measurement.efforts[i] << "\n";
        }
    }

    outputFile.close();
}

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }
 
  enable_torque(false);
  set_control_mode(ControlMode::Position, true);
  set_joint_params();
  enable_torque(true);

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Adding Sync Write Handler for Goal Position Item...");
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Address: %d, Data Length: %d", control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length);

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    // RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to add Sync Write Handler for Goal Position Item: %s", log);
    return CallbackReturn::ERROR;
  }else {
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Sync Write Handler for Goal Position Item added successfully.");
}

// Wyświetl informację o próbie dodania obsługi zapisu synchronicznego dla pozycji zadanej.
RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Adding Sync Write Handler for Goal Velocity Item...");

// Wyświetl adres i długość danych dla pozycji zadanej, które będą używane w obsłudze zapisu synchronicznego.
RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Address: %d, Data Length: %d",
            control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length);

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    // RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to add Sync Write Handler for Goal Velocity Item: %s", log);
    return CallbackReturn::ERROR;
  }else {
  // Jeśli dodanie obsługi zapisu synchronicznego zakończyło się sukcesem, wyświetl informację o sukcesie.
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Sync Write Handler for Goal Velocity Item added successfully.");
}

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length +
                         control_items_[kPresentCurrentItem]->data_length + 2;
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Adding Sync Read Handler with address: %d, length: %d", start_address, read_length);
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Sync Read start adress%s", log);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "OK TO JA SPADAM- mozesz tu generowac wykres csv");
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

    // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Sync Read read1 %s", log);
    // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Control Table Index: %d", kPresentPositionVelocityCurrentIndex);
    // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "IDs Size: %zu", ids.size());
    // std::cout << "ID: " << static_cast<int>(ids.data()[0]) << std::endl;


  if (!dynamixel_workbench_.syncRead(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Sync Read read1 %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentCurrentItem]->address,
        control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Sync Read read2 %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentVelocityItem]->address,
        control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Sync Read read3 %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentPositionItem]->address,
        control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Sync Read read4 %s", log);
  }

  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
    // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Pozycja serwomechanizmu %d: %.2f rad", ids[i], joints_[i].state.position);
    // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Prędkość serwomechanizmu %d: %.2f m/s", ids[i], joints_[i].state.velocity);
    // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Prąd serwomechanizmu %d: %.2f mA", ids[i], joints_[i].state.effort);
  }

    // Get the current time as epoch (in seconds)
  auto now = std::chrono::system_clock::now();
  // Initialize start_time if it's the first measurement
  if (collected_measurements.empty()) {
      start_time = now;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Starting to collect data");
  }

  // Calculate relative time (time since start_time)
  double relative_time_miliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

  // Store collected measurements
  MeasurementData measurement;
  measurement.relative_time_miliseconds = relative_time_miliseconds;
  // Assign processed values to measurement
  for (uint i = 0; i < ids.size(); i++) {
      measurement.positions.push_back(joints_[i].state.position);
      measurement.velocities.push_back(joints_[i].state.velocity);
      measurement.efforts.push_back(joints_[i].state.effort);
  }
  collected_measurements.push_back(measurement);

  // Check if 10 seconds have passed since the first measurement
  if (relative_time_miliseconds >= 10000) {
      saveMeasurementsToFile("data.csv");
      collected_measurements.clear(); // Clear collected measurements after saving
      start_time = now; // Reset start_time for the next batch of measurements
  }

  return return_type::OK;
}

return_type DynamixelHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // Velocity control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.velocity != j.prev_command.velocity; })) {
    set_control_mode(ControlMode::Velocity);
    if(mode_changed_)
    {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }
  
  // Position control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.position != j.prev_command.position; })) {
    set_control_mode(ControlMode::Position);
    if(mode_changed_)
    {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
               joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.effort != 0.0; })) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // if all command values are unchanged, then remain in existing control mode and set corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    default: // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }

}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");
    if(control_mode_ != ControlMode::Velocity)
    {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");
    if(control_mode_ != ControlMode::Position)
    {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }
  
  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    commands[i] = dynamixel_workbench_.convertRadian2Value(
      ids[i], static_cast<float>(joints_[i].command.position));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "set joint positions syncWrite %s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    commands[i] = dynamixel_workbench_.convertVelocity2Value(
      ids[i], static_cast<float>(joints_[i].command.velocity));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "set joint velocities syncWrite %s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
