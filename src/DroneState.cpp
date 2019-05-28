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

#include <sstream>
#include <math.h>
#include "DroneState.hpp"



DroneState::DroneState()
    :  m_quaternionState{}
    ,  m_quatMutex{}
//     ,  m_memoryTime()
//     ,  m_apaTrim(0,0,0)
//     ,  m_calibrationFile("apaCalibration.cal")
//     ,  m_accelerometerReadings()
//     ,  m_magnetometerReadings()
//     ,  m_gyroscopeReadings()
//     ,  m_altimeterReadings()
//     ,  m_temperatureReadings()
{
}

openkorp::logic::Quaternion const DroneState::getQuaternionState()
{
  std::lock_guard<std::mutex> lock(m_quatMutex);
  return m_quaternionState;
}

void DroneState::setQuaternionState(openkorp::logic::Quaternion const &a_quat)
{
  std::lock_guard<std::mutex> lock(m_quatMutex);
  m_quaternionState = a_quat;
}

std::string DroneState::toString()
{
  std::lock_guard<std::mutex> lock(m_quatMutex);
  std::stringstream ss;
  // char qbuffer [50];
  // char ebuffer [50];
  ss << m_quaternionState.ShortName() <<": (" + std::to_string(m_quaternionState.w()) + ", " + std::to_string(m_quaternionState.x()) + ", " + std::to_string(m_quaternionState.y()) + ", " + std::to_string(m_quaternionState.z()) + ")" << std::endl;
  std::vector<double> euler = quaternion2Euler(m_quaternionState);
  ss << "Euler: (" + std::to_string(euler.at(0)) + ", " + std::to_string(euler.at(1)) + ", " + std::to_string(euler.at(2)) + ")" << std::endl;
  // double roll, pitch, yaw;
  // toEulerAngle(msg, roll, pitch, yaw);
  // sprintf(ebuffer, "Euler: (%0.3f, %0.3f, %0.3f)", roll, pitch, yaw); 
  return ss.str();
}

std::vector<double> DroneState::quaternion2Euler(openkorp::logic::Quaternion const &a_quat)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (a_quat.w() * a_quat.x() + a_quat.y() * a_quat.z());
	double cosr_cosp = +1.0 - 2.0 * (a_quat.x() * a_quat.x() + a_quat.y() * a_quat.y());
	double roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (a_quat.w() * a_quat.y() - a_quat.z() * a_quat.x());
  double pitch;
	if (fabs(sinp) >= 1) {
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  } else {
		pitch = asin(sinp);
  }

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (a_quat.w() * a_quat.z() + a_quat.x() * a_quat.y());
	double cosy_cosp = +1.0 - 2.0 * (a_quat.y() * a_quat.y() + a_quat.z() * a_quat.z());  
	double yaw = atan2(siny_cosp, cosy_cosp);
  return std::vector<double>{roll, pitch, yaw};
}