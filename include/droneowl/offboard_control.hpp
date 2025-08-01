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

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
// #include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>
// #include "droneowl/pid.hpp"
// #include "config/waypoints.yaml"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) :
		Node("offboard_control"),
		state_{State::init},
		service_result_{0},
		service_done_{false},
		offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>(px4_namespace+"in/offboard_control_mode", 10)},
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(px4_namespace+"in/trajectory_setpoint", 10)},
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command")}
	{

		/*odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(px4_namespace+"out/vehicle_odometry", 10,
			[this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
				current_odom_ = *msg;
		}); */

		// Define trajectory waypoints
		/*waypoints_ = {
			{0.0f, 0.0f, -5.0f},
			{5.0f, 0.0f, -5.0f},
			{5.0f, 5.0f, -5.0f},
			{0.0f, 5.0f, -5.0f},
			{0.0f, 0.0f, -5.0f}
		}; */

		this->declare_parameter<std::string>("waypoint_file", "waypoints.yaml");
		std::string waypoint_file = this->get_parameter("waypoint_file").as_string();
		RCLCPP_INFO(this->get_logger(), "waypoints file: %s", waypoint_file.c_str());
		load_waypoints_from_yaml(waypoint_file);
		if (waypoints_.empty()) {
			// fallback default as circular
			/*waypoints_ = {
				{0.0f, 0.0f, -5.0f},
				{5.0f, 0.0f, -5.0f},
				{5.0f, 5.0f, -5.0f},
				{0.0f, 5.0f, -5.0f},
				{0.0f, 0.0f, -5.0f}
			};*/
			const float radius = 5.0f;
			const float center_x = 0.0f;
			const float center_y = 0.0f;
			const float z = -5.0f;  // hover at 5m altitude (negative in NED)

			const int num_points = 8;  // number of waypoints around the circle
			for (int i = 0; i < num_points; i++) {
				float angle = 2.0f * M_PI * static_cast<float>(i) / num_points;
				float x = center_x + radius * std::cos(angle);
				float y = center_y + radius * std::sin(angle);
				waypoints_.push_back({x, y, z});
			}

			// Optionally return to the starting point
			waypoints_.push_back({center_x + radius, center_y, z});
		}

		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

	void switch_to_offboard_mode();
	void arm();
	void disarm();

private:
	enum class State{
		init,
		offboard_requested,
		wait_for_stable_offboard_mode,
		arm_requested,
		armed
	} state_;
	uint8_t service_result_;
	bool service_done_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	/*rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
	px4_msgs::msg::VehicleOdometry current_odom_;

	PID pid_x_{1.0, 0.0, 0.3};
	PID pid_y_{1.0, 0.0, 0.3};
	PID pid_z_{1.0, 0.0, 0.3}; */
	std::vector<std::array<float,3>> waypoints_;
	size_t current_wp_{0};


	void load_waypoints_from_yaml(const std::string &filepath);
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void timer_callback(void);
};