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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef DRONESTATE_H
#define DRONESTATE_H

#include <mutex>
#include <vector>
#include <string>
#include "openkorp-message-set.hpp"

class DroneState{
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
  std::vector<double> quaternion2Euler(openkorp::logic::Quaternion const &);
  std::string toString();


 private:

  openkorp::logic::Quaternion m_quaternionState;
  std::mutex m_quatMutex;

};


#endif
