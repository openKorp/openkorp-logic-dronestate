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
    : m_quaternionState{},
      m_motorState{{0.0f, 0.0f, 0.0f, 0.0f}},
      m_quatMutex{}  //     ,  m_memoryTime()
                     //     ,  m_apaTrim(0,0,0)
                     //     ,  m_calibrationFile("apaCalibration.cal")
                     //     ,  m_accelerometerReadings()
                     //     ,  m_magnetometerReadings()
                     //     ,  m_gyroscopeReadings()
                     //     ,  m_altimeterReadings()
                     //     ,  m_temperatureReadings()
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
  // char qbuffer [50];
  ss << std::fixed << std::setprecision(2);
  // char ebuffer [50];
  ss << m_quaternionState.ShortName() << ": (" << m_quaternionState.w() << ", "
     << m_quaternionState.x() << ", " << m_quaternionState.y() << ", "
     << m_quaternionState.z() << ")" << std::endl;
  std::vector<double> euler = quaternion2Euler(m_quaternionState);
  ss << "Euler: (" << euler.at(0) << ", " << euler.at(1) << ", " << euler.at(2)
     << ")" << std::endl;
  // double roll, pitch, yaw;
  // toEulerAngle(msg, roll, pitch, yaw);
  // sprintf(ebuffer, "Euler: (%0.3f, %0.3f, %0.3f)", roll, pitch, yaw);
  ss << "Motor: (" << m_motorState.at(0) << ", " << m_motorState.at(1) << ", "
     << m_motorState.at(2) << ", " << m_motorState.at(3) << ")" << std::endl;

  ss << "\t\t\t\t\t\t\t\t\t\t"
     << "    Front" << std::endl;
  ss << "\t\t\t\t\t\t\t\t\t" << m_motorState.at(0) << "    \t\t"
     << m_motorState.at(3) << std::endl;
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
  ss << "\t\t\t\t\t\t\t\t\t" << m_motorState.at(1) << "    \t\t"
     << m_motorState.at(2) << std::endl;

  return ss.str();
}

std::vector<double> DroneState::quaternion2Euler(
    openkorp::logic::Quaternion const &a_quat) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (a_quat.w() * a_quat.x() + a_quat.y() * a_quat.z());
  double cosr_cosp =
      +1.0 - 2.0 * (a_quat.x() * a_quat.x() + a_quat.y() * a_quat.y());
  double roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (a_quat.w() * a_quat.y() - a_quat.z() * a_quat.x());
  double pitch;
  if (fabs(sinp) >= 1) {
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  } else {
    pitch = asin(sinp);
  }

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (a_quat.w() * a_quat.z() + a_quat.x() * a_quat.y());
  double cosy_cosp =
      +1.0 - 2.0 * (a_quat.y() * a_quat.y() + a_quat.z() * a_quat.z());
  double yaw = atan2(siny_cosp, cosy_cosp);
  return std::vector<double>{roll, pitch, yaw};
}

std::array<float, 4> const DroneState::getMotorState() {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  return {m_motorState.at(0), m_motorState.at(1), m_motorState.at(2),
          m_motorState.at(3)};
}

void DroneState::setMotorState(std::array<float, 4> const a_state) {
  std::lock_guard<std::mutex> lock(m_quatMutex);
  m_motorState = {a_state.at(0), a_state.at(1), a_state.at(2), a_state.at(3)};
}