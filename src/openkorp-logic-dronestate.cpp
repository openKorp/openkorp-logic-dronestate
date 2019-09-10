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
#include <chrono>
#include <iostream>

#include "DroneState.hpp"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "openkorp-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  // int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      // 0 == commandlineArguments.count("input") ||
      0 == commandlineArguments.count("freq")) {
    std::cerr << argv[0] << " is the flight controller for the drone."
              << std::endl;
    std::cerr << "Incorrect params input." << std::endl;
    // std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session>
    // [--verbose] --input=<js node>" << std::endl; std::cerr << "Example: " <<
    // argv[0] << " --cid=111 --input=/dev/input/js0" << std::endl; retCode = 1;
    return 1;
  }
  DroneState state{};

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

  auto onQuaternionMsg{[&od4, &VERBOSE,
                        &state](cluon::data::Envelope &&envelope) {
    auto msg =
        cluon::extractMessage<openkorp::logic::Quaternion>(std::move(envelope));
    state.setQuaternionState(msg);
  }};

  od4.dataTrigger(openkorp::logic::Quaternion::ID(), onQuaternionMsg);

  auto timeStamp = std::chrono::high_resolution_clock::now();

  auto atFrequency{[&VERBOSE, &state, &timeStamp]() -> bool {
    auto newtimeStamp = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> timeElapsed = newtimeStamp - timeStamp;
    if (VERBOSE == 2) {
      mvprintw(0, 0, state.toString().c_str());
      mvprintw(10, 0, std::to_string(1 / timeElapsed.count()).c_str());
      refresh();
    }
    timeStamp = newtimeStamp;
    return true;
  }};

  od4.timeTrigger(FREQ, atFrequency);

  if (VERBOSE == 2) {
    endwin();
  }

  return 0;
}
