/*
 * Copyright (C) 2019 Björnborg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <ncurses.h>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
// #include "Ps3Controller.h"
#include "openkorp-message-set.hpp"

static void toEulerAngle(const openkorp::logic::Quaternion &q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny_cosp, cosy_cosp);
}

int32_t main(int32_t argc, char **argv) {
  // int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      // 0 == commandlineArguments.count("input") ||
      0 == commandlineArguments.count("freq")
      ) {
    // std::cerr << argv[0] << " interfaces to the ps3controller for the Drone." << std::endl;
    // std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--verbose] --input=<js node>" << std::endl;
    // std::cerr << "Example: " << argv[0] << " --cid=111 --input=/dev/input/js0" << std::endl;
    // retCode = 1;
    return 1;
  } 
  
  // Ps3Controller ps3controller();
  int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
  if (VERBOSE) {
    VERBOSE = std::stoi(commandlineArguments["verbose"]);
  }
  uint16_t const CID = std::stoi(commandlineArguments["cid"]);

  cluon::OD4Session od4{CID};

  float const FREQ = std::stof(commandlineArguments["freq"]);
  
  if (VERBOSE == 2) {
    initscr();
  }

  auto onQuaternionMsg{[&od4, &VERBOSE](cluon::data::Envelope &&envelope)
    {
      auto msg = cluon::extractMessage<openkorp::logic::Quaternion>(
            std::move(envelope));
      // std::array<double, 4> quaternion{msg.w(), msg.x(), msg.y(), msg.z()};
        
      // od4.send(msg, cluon::time::now(), 0);
      char qbuffer [50];
      char ebuffer [50];
      sprintf(qbuffer, "Quaternion: (%0.3f, %0.3f, %0.3f, %0.3f)", msg.w(), msg.x(), msg.y(), msg.z());
      double roll, pitch, yaw;
      toEulerAngle(msg, roll, pitch, yaw);
      sprintf(ebuffer, "Euler: (%0.3f, %0.3f, %0.3f)", roll, pitch, yaw); 
      if (VERBOSE == 2) {
        mvprintw(1, 1, qbuffer); 
        mvprintw(2, 1, ebuffer); 
        refresh();   
      }
    }};
  
  od4.dataTrigger(openkorp::logic::Quaternion::ID(), onQuaternionMsg);

  auto atFrequency{[]() -> bool
  {
    return true;
  }};
  

  od4.timeTrigger(FREQ, atFrequency);
  if (VERBOSE == 2) {
    endwin();
  }

  return 0;
}
