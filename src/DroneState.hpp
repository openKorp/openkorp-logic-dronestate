/**
 * Drone state - gathers external data to compute the state of the drone
 * Copyright (C) 2017 openKorp
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef DRONESTATE_H
#define DRONESTATE_H

#include <array>
#include <eigen/Dense>
#include <mutex>
#include <string>
#include <vector>
#include "openkorp-message-set.hpp"

class DroneState {
 private:
  DroneState(DroneState const &) = delete;
  DroneState(DroneState &&) = delete;
  DroneState &operator=(DroneState const &) = delete;
  DroneState &operator=(DroneState &&) = delete;

 public:
  DroneState();
  ~DroneState() = default;

  openkorp::logic::Quaternion const getQuaternionState();
  void setQuaternionState(openkorp::logic::Quaternion const &);
  Eigen::Vector3f quaternion2Euler(openkorp::logic::Quaternion const &);
  std::string toString();
  std::array<float, 4> const getMotorState();
  void setMotorState(std::array<float, 4> const &);
  void setInstruction(std::array<float, 4> const &);
  void calcPID();

 private:
  std::mutex m_quatMutex;
  openkorp::logic::Quaternion m_quaternionState;
  // Motor esc signal 1,2,3,4
  Eigen::Vector4f m_motorState;
  // Instruction in
  // roll     [rad]
  // pitch    [rad]
  // yaw      [rad/s]
  // throttle [percent]
  Eigen::Vector4f m_instruction;
  // Gains in
  // Roll
  //  Kp Ki Kd
  // Pitch
  //  Kp Ki Kd
  // Yaw
  //  Kp Ki Kd
  Eigen::Matrix3f m_gain;
  // Error matrix
  //    Current       Sum           Diff prev
  //    Roll_error    Roll_error_i  Roll_error_d
  //    Pitch_error   Pitch_error_i Pitch_error_d
  //    Yaw_error     Yaw_error_i   Yaw_error_d
  Eigen::Matrix3f m_error;

  Eigen::Vector3f m_controlPid;
};

#endif
