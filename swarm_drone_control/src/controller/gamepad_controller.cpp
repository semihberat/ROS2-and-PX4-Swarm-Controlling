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

#include "swarm_drone_control/gamepad_controller.hpp"

// === CONSTRUCTOR ===
OffboardControl::OffboardControl() : LifecycleNode("offboard_control")
{
    this->declare_parameter("sys_id", 1);
}

// === LIFECYCLE CALLBACKS ===
LifecycleCallbackReturn OffboardControl::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    int sys_id = this->get_parameter("sys_id").as_int();

    std::string OFFBOARD_CONTROL_MODE = "/px4_" + std::to_string(sys_id) + "/fmu/in/offboard_control_mode";
    std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
    std::string VEHICLE_COMMAND = "/px4_" + std::to_string(sys_id) + "/fmu/in/vehicle_command";
    std::string VEHICLE_ATTITUDE = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_attitude";
    std::string VEHICLE_STATUS = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_status_v1";

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(OFFBOARD_CONTROL_MODE, 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(VEHICLE_COMMAND, 10);

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&OffboardControl::joy_subscriber_callback, this, _1));
    vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        VEHICLE_ATTITUDE, qos, std::bind(&OffboardControl::listen_to_altitude, this, _1));
    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        VEHICLE_STATUS, qos, std::bind(&OffboardControl::vehicle_status_listener_callback, this, _1));

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void
    {
        if (joy_msgs == nullptr)
            return;
        // Sol joystick sağ-aşağıda tutulursa arm et
        if (joy_msgs->axes[0] < -0.5f && joy_msgs->axes[1] < -0.5f && !this->vehicle_status_->arming_state)
        {
            int arm_timer = 0;
            arm_timer++;
            RCLCPP_INFO(this->get_logger(), "[ARM] Holding: %d/20, axes[0]=%.2f, axes[1]=%.2f",
                        arm_timer, joy_msgs->axes[0], joy_msgs->axes[1]);
            if (arm_timer > 20)
            {
                RCLCPP_WARN(this->get_logger(), "*** SENDING ARM COMMAND ***");
                this->arm();
                arm_timer = 0;
            }
        }
        else
        {
            // Pozisyon bırakılırsa sıfırla
        }
        this->relative_movement(joy_msgs->axes[4] * 10, -joy_msgs->axes[3] * 10, -joy_msgs->axes[1] * 10, -joy_msgs->axes[0] * 3.14f);
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
    timer_->cancel();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn OffboardControl::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE");

    offboard_control_mode_publisher_->on_activate();
    trajectory_setpoint_publisher_->on_activate();
    vehicle_command_publisher_->on_activate();

    timer_->reset();

    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn OffboardControl::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE");

    offboard_control_mode_publisher_->on_deactivate();
    trajectory_setpoint_publisher_->on_deactivate();
    vehicle_command_publisher_->on_deactivate();

    timer_->cancel();

    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn OffboardControl::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");

    offboard_control_mode_publisher_.reset();
    trajectory_setpoint_publisher_.reset();
    vehicle_command_publisher_.reset();
    joy_subscriber_.reset();
    vehicle_attitude_subscriber_.reset();
    vehicle_status_subscriber_.reset();

    if (timer_)
    {
        timer_->cancel();
        timer_.reset();
    }

    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn OffboardControl::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");

    offboard_control_mode_publisher_.reset();
    trajectory_setpoint_publisher_.reset();
    vehicle_command_publisher_.reset();
    joy_subscriber_.reset();
    vehicle_attitude_subscriber_.reset();
    vehicle_status_subscriber_.reset();

    if (timer_)
    {
        timer_->cancel();
        timer_.reset();
    }

    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn OffboardControl::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_ERROR(this->get_logger(), "ON_ERROR");

    if (timer_)
    {
        timer_->cancel();
        timer_.reset();
    }
    offboard_control_mode_publisher_.reset();
    trajectory_setpoint_publisher_.reset();
    vehicle_command_publisher_.reset();

    rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
    return LifecycleCallbackReturn::FAILURE;
}

// === PUBLIC MEMBER FUNCTIONS ===

// === PUBLIC MEMBER FUNCTIONS ===
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

// === PRIVATE MEMBER FUNCTIONS ===
void OffboardControl::joy_subscriber_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joy_msgs = msg;
}

void OffboardControl::vehicle_status_listener_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    vehicle_status_ = msg;
}

void OffboardControl::relative_movement(float v_x, float v_y, float v_z, float yawspeed)
{
    // Kuaterniyondan yaw açısını hesapla (drone'un yatay düzlemdeki dönüş açısı)
    float q_yaw = atan2f(2.0f * (vehicle_attitude_->q[0] * vehicle_attitude_->q[3] + vehicle_attitude_->q[1] * vehicle_attitude_->q[2]),
                         1.0f - 2.0f * (vehicle_attitude_->q[2] * vehicle_attitude_->q[2] + vehicle_attitude_->q[3] * vehicle_attitude_->q[3]));

    // Joystick komutlarını drone'un yönüne göre dünya koordinatlarına çevir
    float v_n = v_x * cosf(q_yaw) - v_y * sinf(q_yaw); // Kuzey yönündeki hız
    float v_e = v_x * sinf(q_yaw) + v_y * cosf(q_yaw); // Doğu yönündeki hız

    publish_trajectory_setpoint(v_n, v_e, v_z, yawspeed);
}

void OffboardControl::listen_to_altitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    vehicle_attitude_ = msg;
}

void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yawspeed)
{
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {x, y, z};
    msg.yaw = NAN;
    msg.yawspeed = yawspeed;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = this->get_parameter("sys_id").as_int() + 1;
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
    rclcpp::spin(std::make_shared<OffboardControl>()->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
