/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
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
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std::placeholders;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&OffboardControl::joy_subscriber_callback, this, _1));
		vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControl::listen_to_altitude, this, _1));
		vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			"/fmu/out/vehicle_status_v1", qos, std::bind(&OffboardControl::vehicle_status_listener_callback, this, _1));
		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void
		{
			if (offboard_setpoint_counter_ == 10)
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}

			publish_offboard_control_mode();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11)
			{
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	std::vector<float> q = std::vector<float>(4, 0.0f); //!< vehicle attitude quaternion
	rclcpp::TimerBase::SharedPtr timer_;
	int8_t arm_timer = 0;
	bool is_armed = false;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
	std::atomic<uint64_t> timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yawspeed);
	void joy_subscriber_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void relative_movement(float v_x, float v_y, float v_z, float yawspeed);
	void listen_to_altitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
	void vehicle_status_listener_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
};

void OffboardControl::joy_subscriber_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	// Sol joystick sağ-aşağıda tutulursa arm et
	if (msg->axes[0] < -0.5f && msg->axes[1] < -0.5f && !this->is_armed)
	{
		arm_timer++;
		RCLCPP_INFO(this->get_logger(), "[ARM] Holding: %d/20, armed=%d, axes[0]=%.2f, axes[1]=%.2f", 
			arm_timer, is_armed, msg->axes[0], msg->axes[1]);
		if (arm_timer > 20)
		{ // 20 mesaj @ ~10Hz = ~2 saniye
			RCLCPP_WARN(this->get_logger(), "*** SENDING ARM COMMAND ***");
			this->arm();
			arm_timer = 0;
		}
	}
	else
	{
		arm_timer = 0; // Pozisyon bırakılırsa sıfırla
	}
	this->relative_movement(msg->axes[1] * 10, -msg->axes[0] * 10, -msg->axes[4] * 10, -msg->axes[3] * 3.14f); // x, y, z, yawspeed
																											  // Button B to disarm
}

void OffboardControl::vehicle_status_listener_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
	bool prev_armed = this->is_armed;
	this->is_armed = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
	
	// Her 50 mesajda bir durum yazdır
	static int msg_count = 0;
	msg_count++;
	if(msg_count % 50 == 0 || prev_armed != this->is_armed) {
		RCLCPP_INFO(this->get_logger(), "[STATUS] arming_state=%d, is_armed=%d", 
			msg->arming_state, this->is_armed);
	}
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */

void OffboardControl::relative_movement(float v_x, float v_y, float v_z, float yawspeed)
{
	// Kuaterniyondan yaw açısını hesapla (drone'un yatay düzlemdeki dönüş açısı)
	float q_yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
	
	// Joystick komutlarını drone'un yönüne göre dünya koordinatlarına çevir
	// Örnek: Joystick ileri → drone hangi yöne bakıyorsa o yöne git
	// Body frame (vücut koordinatı) → NED frame (dünya koordinatı) dönüşümü
	float v_n = v_x * cosf(q_yaw) - v_y * sinf(q_yaw);  // Kuzey yönündeki hız
	float v_e = v_x * sinf(q_yaw) + v_y * cosf(q_yaw);  // Doğu yönündeki hız
	
	publish_trajectory_setpoint(v_n, v_e, v_z, yawspeed);
};

void OffboardControl::listen_to_altitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
	this->q.assign(msg->q.begin(), msg->q.end());
};

void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yawspeed)
{
	TrajectorySetpoint msg{};
	msg.position = {NAN, NAN, NAN}; // Position NAN olmali ki PX4 velocity'yi kullansın
	msg.velocity = {x, y, z};		// Velocity in NED frame [m/s]
	msg.yaw = NAN;					// Yaw NAN olmali ki yawspeed'i kullansın
	msg.yawspeed = yawspeed;		// [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
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
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
