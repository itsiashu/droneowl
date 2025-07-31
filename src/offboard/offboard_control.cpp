/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples * 
 * @author Beniamino Pozzan <beniamino.pozzan@gmail.com>
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <Eigen/Dense>

#include "droneowl/offboard_control.hpp"

/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void OffboardControl::load_waypoints_from_yaml(const std::string &filepath)
{
	waypoints_.clear();
	try {
		YAML::Node config = YAML::LoadFile(filepath);

		if (!config["waypoints"]) {
			RCLCPP_ERROR(this->get_logger(), "YAML file missing 'waypoints' field");
			return;
		}

		for (const auto &node : config["waypoints"]) {
			if (node.IsSequence() && node.size() == 3) {
				float x = node[0].as<float>();
				float y = node[1].as<float>();
				float z = node[2].as<float>();
				waypoints_.push_back({x, y, z});
			}
		}

		RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from YAML file: %s",
					waypoints_.size(), filepath.c_str());
	}
	catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file %s: %s",
					 filepath.c_str(), e.what());
	}
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/*void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;   // enable velocity mode
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}*/


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
/*void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}*/

void OffboardControl::publish_trajectory_setpoint()
{
	// Current target waypoint
	auto target = waypoints_[current_wp_];

	static int count = 0;
	count++;

	// Move to the next waypoint after ~5 seconds (50 timer ticks at 100ms)
	if (count > 50) {
		count = 0;
		current_wp_ = (current_wp_ + 1) % waypoints_.size();
		target = waypoints_[current_wp_];
		RCLCPP_INFO(this->get_logger(),
					"Switching to waypoint %zu: (%.1f, %.1f, %.1f)",
					current_wp_, target[0], target[1], target[2]);
	}

	// Publish position setpoint
	TrajectorySetpoint msg{};
	msg.position = {target[0], target[1], target[2]};
	msg.velocity = {NAN, NAN, NAN};  // leave velocities unused
	msg.yaw = 0.0f;                  // face forward
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::timer_callback(void){
	static uint8_t num_of_steps = 0;

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	switch (state_)
	{
	case State::init :
		switch_to_offboard_mode();
		state_ = State::offboard_requested;
		break;
	case State::offboard_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
				state_ = State::wait_for_stable_offboard_mode;				
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps > 10){
			arm();
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "vehicle is armed");
				state_ = State::armed;
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	default:
		break;
	}
}

void OffboardControl::response_callback(
      rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
	  auto reply = future.get()->reply;
	  service_result_ = reply.result;
      switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}
