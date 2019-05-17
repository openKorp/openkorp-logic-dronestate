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

#ifndef LOGIC_MINIATURE_DRONESTATE_H
#define LOGIC_MINIATURE_DRONESTATE_H

// #include <map>
// #include <vector>
// #include <memory>
// #include <eigen3/Eigen/Dense>
// #include <string>

// #include <opendavinci/odcore/base/Mutex.h>
// #include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

// #include <opendavinci/odcore/data/Container.h>
// #include <odvdimu/GeneratedHeaders_ODVDIMU.h>

// namespace opendlv {
// namespace logic {
// namespace miniature {

// class DroneState : 
//   public odcore::base::module::TimeTriggeredConferenceClientModule {
//  public:
//   DroneState(const int &, char **);
//   DroneState(const DroneState &) = delete;
//   DroneState &operator=(const DroneState &) = delete;
//   virtual ~DroneState();
//   virtual void nextContainer(odcore::data::Container &);

//  private:
//   void setUp();
//   void tearDown();
//   virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

//   void RemoveOldData();
//   int8_t saveApaCalibrationFile();
//   int8_t loadApaCalibrationFile();
//   Eigen::Vector3f GetAircraftPrincipalAxesValues(Eigen::Vector3f) const;
//   Eigen::Vector3f GetAccelerationLatest();
//   Eigen::Vector3f GetAccelerationAvg();
//   Eigen::Vector3f GetMagneticfieldLatest();
//   Eigen::Vector3f GetMagneticfieldAvg();
//   Eigen::Vector3f GetAngularVelocityLatest();
//   Eigen::Vector3f GetAngularVelocityAvg();
//   float GetTemperatureLatest();
//   float GetTemperatureAvg();
//   float GetAltitudeLatest();
//   float GetAltitudeAvg();

//   bool m_initialised;
//   odcore::base::Mutex m_mutex;
//   float m_memoryTime;
//   Eigen::Vector3f m_apaTrim;
//   std::string const m_calibrationFile;
//   std::map<int32_t, std::vector<odcore::data::Container>> m_accelerometerReadings;
//   std::map<int32_t, std::vector<odcore::data::Container>> m_magnetometerReadings;
//   std::map<int32_t, std::vector<odcore::data::Container>> m_gyroscopeReadings;
//   std::map<int32_t, std::vector<odcore::data::Container>> m_altimeterReadings;
//   std::map<int32_t, std::vector<odcore::data::Container>> m_temperatureReadings;
// };

// }
// }
// }

#endif
