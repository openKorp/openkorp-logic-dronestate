/**
 * Copyright (C) 2019 Björnborg
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

// #include <iostream>
// #include <fstream>
// #include <cmath>

// #include "openkorp-message-set.hpp"

#include "DroneState.hpp"
#include <math.h>
#include <iomanip>
#include <sstream>

DroneState::DroneState()
    : m_quatMutex{},
      m_quaternionState{},
      m_motorState({0.0f, 0.0f, 0.0f, 0.0f}),
      m_instruction({0.0f, 0.0f, 0.0f, 0.0f}),
      m_gain(m_gain.Zero()),
      m_error(m_error.Zero()),
      m_controlPid(m_controlPid.Zero())

{}

openkorp::logic::Quaternion const DroneState::getQuaternionState() {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  return m_quaternionState;
}

void DroneState::setQuaternionState(openkorp::logic::Quaternion const &a_quat) {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  m_quaternionState = a_quat;
}

std::string DroneState::toString() {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  std::stringstream ss;
  // Eigen::IOFormat CommaInitFmtMatrix(Eigen::StreamPrecision,
  //                                    Eigen::DontAlignCols, ", ", "\n", "",
  //                                    "",
  //                                    "\n", "");
  Eigen::IOFormat CommaInitFmtVectors(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "", "", "", "", "");
  ss << std::fixed << std::setprecision(2);
  ss << "Gains:" << std::endl;
  ss << m_gain << std::endl;
  ss << m_quaternionState.ShortName() << ": (" << m_quaternionState.w() << ", "
     << m_quaternionState.x() << ", " << m_quaternionState.y() << ", "
     << m_quaternionState.z() << ")" << std::endl;
  //   std::vector<double> euler = quaternion2Euler(m_quaternionState);
  ss << "Euler:"
     << quaternion2Euler(m_quaternionState).format(CommaInitFmtVectors)
     << std::endl;
  ss << "Instruction: " << m_instruction.format(CommaInitFmtVectors)
     << std::endl;
  ss << "Error: " << m_error << std::endl;
  ss << "Pid control: " << m_controlPid.format(CommaInitFmtVectors)
     << std::endl;
  ss << "Motor: " << m_motorState.format(CommaInitFmtVectors) << std::endl;

  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "    Front" << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t"
     << "1    \t\t\t4" << std::endl;

  ss << "\t\t\t\t\t\t\t\t\t" << m_motorState(0) << "    \t\t" << m_motorState(3)
     << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "()        ()" << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "  \\      /  " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "   \\    /   " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "    \\  /    " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "     QQ     " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "    /  \\    " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "   /    \\   " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "  /      \\  " << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "()        ()" << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t" << m_motorState(1) << "    \t\t" << m_motorState(2)
     << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t"
     << "2    \t\t\t3" << std::endl;

  return ss.str();
}

Eigen::Vector3f DroneState::quaternion2Euler(
    openkorp::logic::Quaternion const &a_quat) {
  // roll (x-axis rotation)
  float sinr_cosp = +2.0f * (a_quat.w() * a_quat.x() + a_quat.y() * a_quat.z());
  float cosr_cosp =
      +1.0f - 2.0f * (a_quat.x() * a_quat.x() + a_quat.y() * a_quat.y());
  float roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = +2.0f * (a_quat.w() * a_quat.y() - a_quat.z() * a_quat.x());
  float pitch;
  if (fabs(sinp) >= 1) {
    pitch =
        copysign(float(M_PI) / 2.0f, sinp);  // use 90 degrees if out of range
  } else {
    pitch = asin(sinp);
  }

  // yaw (z-axis rotation)
  float siny_cosp = +2.0f * (a_quat.w() * a_quat.z() + a_quat.x() * a_quat.y());
  float cosy_cosp =
      +1.0f - 2.0f * (a_quat.y() * a_quat.y() + a_quat.z() * a_quat.z());
  float yaw = atan2(siny_cosp, cosy_cosp);
  Eigen::Vector3f eulerAngles;
  eulerAngles << roll, pitch, yaw;
  return eulerAngles;
}

std::array<float, 4> const DroneState::getMotorState() {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  return {m_motorState(0), m_motorState(1), m_motorState(2), m_motorState(3)};
}

void DroneState::setMotorState(std::array<float, 4> const &a_state) {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  m_motorState = {a_state.at(0), a_state.at(1), a_state.at(2), a_state.at(3)};
}

void DroneState::setInstruction(std::array<float, 4> const &a_instruction) {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  m_instruction = {a_instruction.at(0), a_instruction.at(1),
                   a_instruction.at(2), a_instruction.at(3)};
}

void DroneState::calcPID() {
  //   if controller not active, dont do anything
  if (m_instruction(3) <= 0) return;
  std::lock_guard<std::mutex> lock(m_quatMutex);
  //   save previous error values
  Eigen::Vector3f previousError;
  previousError << m_error.col(0);
  //   integral error
  m_error.col(1) = (m_error.col(1) + m_error.col(0)).eval();
  //   new current error values
  m_error.col(0) =
      m_instruction.head<3>() - quaternion2Euler(m_quaternionState);
  //   derivative error
  m_error.col(2) = m_error.col(1) - previousError;

  // calculate control signal
  m_controlPid = (m_gain.array() * m_error.array()).rowwise().sum();

  //   m_motorState =
}