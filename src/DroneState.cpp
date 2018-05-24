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


#include <iostream>
#include <fstream>
#include <cmath>

#include "openkorp-message-set.hpp"

#include "DroneState.h"

namespace opendlv {
namespace logic {
namespace miniature {


DroneState::DroneState(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, 
          "logic-miniature-dronestate")
    ,  m_initialised(false)
    ,  m_mutex()
    ,  m_memoryTime()
    ,  m_apaTrim(0,0,0)
    ,  m_calibrationFile("apaCalibration.cal")
    ,  m_accelerometerReadings()
    ,  m_magnetometerReadings()
    ,  m_gyroscopeReadings()
    ,  m_altimeterReadings()
    ,  m_temperatureReadings()
{
}

DroneState::~DroneState() 
{
}

void DroneState::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_memoryTime = kv.getValue<float>(getName() + ".memoryTime");

  m_initialised = true;
}

void DroneState::tearDown()
{
  saveApaCalibrationFile();
}

void DroneState::nextContainer(odcore::data::Container &a_c)
{
  if (a_c.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {
    odcore::base::Lock l(m_mutex);
    m_accelerometerReadings[a_c.getSenderStamp()].push_back(a_c);
  } else if (a_c.getDataType() == opendlv::proxy::MagnetometerReading::ID()) {
    odcore::base::Lock l(m_mutex);
    m_magnetometerReadings[a_c.getSenderStamp()].push_back(a_c);
  } else if (a_c.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {
    odcore::base::Lock l(m_mutex);
    m_gyroscopeReadings[a_c.getSenderStamp()].push_back(a_c);
  } else if (a_c.getDataType() == opendlv::proxy::AltimeterReading::ID()) {
    odcore::base::Lock l(m_mutex);
    m_altimeterReadings[a_c.getSenderStamp()].push_back(a_c);
  } else if (a_c.getDataType() == opendlv::proxy::TemperatureReading::ID()) {
    odcore::base::Lock l(m_mutex);
    m_temperatureReadings[a_c.getSenderStamp()].push_back(a_c);
  } else if (a_c.getDataType() == opendlv::logic::StateRequest::ID()) {
    opendlv::logic::StateRequest sr = a_c.getData<opendlv::logic::StateRequest>();
    float rollIncrement = sr.getRollTrim();
    float pitchIncrement = sr.getPitchTrim();
    Eigen::Vector3f inc(rollIncrement, pitchIncrement, 0);
    odcore::base::Lock l(m_mutex);
    m_apaTrim += inc;
  }
  
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DroneState::body()
{

  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "","");
  if (loadApaCalibrationFile() < 0){
    uint count = 0;
    while (count < 500 && getModuleStateAndWaitForRemainingTimeInTimeslice() == 
        odcore::data::dmcp::ModuleStateMessage::RUNNING) {
      count++;
    }
    m_apaTrim = GetAircraftPrincipalAxesValues(GetAccelerationAvg());
  }


  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    RemoveOldData();
    // Eigen::Vector3f accLatest = GetAccelerationLatest();
    Eigen::Vector3f accAvg = GetAccelerationAvg();
    // Eigen::Vector3f magFieLatest = GetMagneticfieldLatest();
    // Eigen::Vector3f magFieAvg = GetMagneticfieldAvg();
    // Eigen::Vector3f angVelLatest = GetAngularVelocityLatest();
    Eigen::Vector3f angVelAvg = GetAngularVelocityAvg();
    // float tempLatest = GetTemperatureLatest();
    // float tempAvg = GetTemperatureAvg();
    // float altLatest = GetAltitudeLatest();
    // float altAvg = GetAltitudeAvg();
    // Eigen::Vector3f apaLatest = GetAircraftPrincipalAxesValues(accLatest) - m_apaTrim;
    Eigen::Vector3f apaAvg = GetAircraftPrincipalAxesValues(accAvg) - m_apaTrim;

    // message opendlv.logic.State [id = 1010] {
    //   float roll [id = 1];
    //   float pitch [id = 2];
    //   float rollSpeed [id = 3];
    //   float pitchSpeed [id = 4];
    // }

    opendlv::logic::State state(apaAvg[0], apaAvg[1], angVelAvg[0], angVelAvg[1]);

    // Eigen::Vector3f apa = GetAircraftPrincipalAxesValues(accLatest);

    // if (getVerbosity() > 0) {
      std::cout << "[" << getName() << "] Average over sensors and time (" << m_memoryTime 
          << " secs)\n" 
          << " State: " << state.toString() << std::endl;
    //       << "Acceleration: " << accLatest.format(fmt) << std::endl
    //       << "Acceleration avg: " << accAvg.format(fmt) << std::endl
    //       << "Magnetic Field: " << magFieLatest.format(fmt) << std::endl
    //       << "Magnetic Field avg: " << magFieAvg.format(fmt) << std::endl
    //       << "Angular velocity: " << angVelLatest.format(fmt) << std::endl
    //       << "Angular velocity avg: " << angVelAvg.format(fmt) << std::endl
    //       << "Temperature: " << tempLatest << std::endl
    //       << "Temperature avg: " << tempAvg << std::endl
    //       << "Altitude: " << altLatest << std::endl
    //       << "Altitude avg: " << altAvg << std::endl
    //       << "Apa1: " << (apaLatest).format(fmt) << std::endl
    //       << "Apa2: " << (apaAvg).format(fmt) << std::endl;
    // }
        
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

int8_t DroneState::saveApaCalibrationFile()
{

  std::ofstream apaCalibrationFile(m_calibrationFile);
  // apaCalibrationFile.open(m_calibrationFile);
  if (apaCalibrationFile.is_open()) {
    apaCalibrationFile << m_apaTrim[0] << "\n" << m_apaTrim[1] << "\n" << m_apaTrim[2] << "\n";
    std::cout << "[" << getName() << "] Saved calibration file: " + m_calibrationFile  << std::endl;;
    
  } else {
    std::cout << "[" << getName() << "] Unable to save calibration file. Tried to open: " + m_calibrationFile + "\n";
    return -1;
  }
  apaCalibrationFile.flush();
  apaCalibrationFile.close();
  return 0;
}


int8_t DroneState::loadApaCalibrationFile() 
{
  std::vector<float> val;
  std::ifstream file(m_calibrationFile, std::ifstream::in);
  if (file.is_open()){
    std::string line;
    for (uint8_t i = 0; i < 3; ++i) {
      std::getline(file, line);
      try{
        val.push_back(std::stof(line));
      } catch (std::invalid_argument e) {
        std::cout << "[" << getName() << "] Invalid calibration file format." << std::endl;
        file.close();
        return -1;
      }
    }

    std::cout << "[" << getName() << "] Loaded the calibration settings." << std::endl;
    std::cout << "\nLoaded:"
        << " Gyro: " << val.at(0) << ", " 
        << val.at(1) << ", " 
        << val.at(2) << std::endl;
    file.close();
    m_apaTrim << val.at(0), val.at(1), val.at(2);
    return 0;
  } else {
    std::cout << "[" << getName() << "] Could not load the calibration settings. Tried to open: " + m_calibrationFile
        << std::endl;
    file.close();
    return -1;
  }
}


void DroneState::RemoveOldData()
{
  odcore::base::Lock l(m_mutex);
  
  odcore::data::TimeStamp now;

  std::vector<std::map<int32_t, std::vector<odcore::data::Container>>> 
      dataTypes;
  dataTypes.push_back(m_accelerometerReadings);
  dataTypes.push_back(m_magnetometerReadings);
  dataTypes.push_back(m_gyroscopeReadings);
  dataTypes.push_back(m_altimeterReadings);
  dataTypes.push_back(m_temperatureReadings);

  for (auto dataType = dataTypes.begin(); dataType != dataTypes.end(); 
      ++dataType) {
    std::map<int32_t , std::vector<odcore::data::Container>>::iterator 
        dataRange = (*dataType).begin();
    while (dataRange != (*dataType).end()) {
      std::vector<odcore::data::Container>::iterator data = 
          (*dataRange).second.begin();
      std::vector<odcore::data::Container> saveData;
      while (data != (*dataRange).second.end()) {
        float timeDifference = 
            (now-(*data).getSentTimeStamp()).toMicroseconds() / 1000000.0;
        if (timeDifference < m_memoryTime) {
          saveData.push_back(*data);
          // (*dataRange).second.erase(data);
        }
        ++data;
      }
      (*dataRange).second = saveData;
      ++dataRange;
    }
  }
  m_accelerometerReadings = dataTypes.at(0);
  m_magnetometerReadings = dataTypes.at(1);
  m_gyroscopeReadings = dataTypes.at(2);
  m_altimeterReadings = dataTypes.at(3);
  m_temperatureReadings = dataTypes.at(4);
}

Eigen::Vector3f DroneState::GetAircraftPrincipalAxesValues(Eigen::Vector3f a_acc) const
{
  Eigen::Vector3f aircraftPrincipalAxes;
  float rollAcc = atan2(a_acc(1), a_acc(2));
  // float lPAcc = 0.01;
  // float hPacc = 1 - lPAcc;
  // Make sure that roll is within [-pi,pi]
  while (rollAcc < -static_cast<float>(M_PI)) {
    rollAcc += 2 * static_cast<float>(M_PI);
  }
  while (rollAcc > static_cast<float>(M_PI)) {
    rollAcc -= 2 * static_cast<float>(M_PI);
  }

  float pitchAcc = atan2(-a_acc(0), 
      sqrt(a_acc(1) * a_acc(1) + a_acc(2) * a_acc(2)));

  while (pitchAcc < -static_cast<float>(M_PI)) {
    pitchAcc += 2 * static_cast<float>(M_PI);
  }
  while (pitchAcc > static_cast<float>(M_PI)) {
    pitchAcc -= 2 * static_cast<float>(M_PI);
  }

  // roll = hPacc * (roll + rollspeed) + (lPAcc * rollAcc);
  // pitch = hPacc * (pitch + pitchspeed) + (lPAcc * pitchAcc);

  // // float heading = atan2(magneticField.at(1), magneticField.at(0));

  // // if(m_debug){
  // //   std::cout << "Altitude: " << altitude << std::endl
  // //       << "Heading: " << 180 * heading / static_cast<float>(M_PI) << std::endl;
  // // }

  // Tilt compensation
  // // magneticField.at(0) = magneticField.at(0) * cosf(pitch) 
  // //     + magneticField.at(2) * sinf(pitch);
  // // magneticField.at(1) = magneticField.at(0) * sinf(pitch) * sinf(roll) 
  // //     + magneticField.at(1) * cosf(roll) 
  // //     - magneticField.at(2) * sinf(roll) * cosf(pitch);
  // // magneticField.at(2) = -magneticField.at(0) * cosf(roll) * sinf(pitch) 
  // //     + magneticField.at(1) * sinf(roll) 
  // //     + magneticField.at(2) * cosf(roll) * cosf(pitch);

  // // heading = atan2(magneticField.at(1),magneticField.at(0));


  // // if(m_debug){
  // //   std::cout << "Tilt compensated: "<< 180 * heading / static_cast<float>(M_PI) << " (Pitch, Roll): " << pitch << "," << roll <<std::endl;
// // }
  aircraftPrincipalAxes << rollAcc, pitchAcc, 0.0;
  return aircraftPrincipalAxes;
}

// Asssumes that all reference axles are the same
Eigen::Vector3f DroneState::GetAccelerationLatest()
{
  odcore::base::Lock l(m_mutex);
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorAcc = m_accelerometerReadings.begin(); 
      sensorAcc != m_accelerometerReadings.end(); ++sensorAcc, ++counter) {
    if (!(*sensorAcc).second.empty()) {
      odcore::data::Container dataContainer = (*sensorAcc).second.back();
      opendlv::proxy::AccelerometerReading ar = 
          dataContainer.getData<opendlv::proxy::AccelerometerReading>();
      Eigen::Vector3f v;
      v << ar.getAccelerationX(), ar.getAccelerationY(), 
          ar.getAccelerationZ();
      sum += v;   
    }
  }
  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

Eigen::Vector3f DroneState::GetAccelerationAvg()
{
  odcore::base::Lock l(m_mutex);
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorAcc = m_accelerometerReadings.begin(); 
      sensorAcc != m_accelerometerReadings.end(); ++sensorAcc) {
    for (std::vector<odcore::data::Container>::iterator dataContainer = 
        (*sensorAcc).second.begin(); dataContainer != (*sensorAcc).second.end(); 
        ++dataContainer, ++counter) {
      opendlv::proxy::AccelerometerReading ar = 
          (*dataContainer).getData<opendlv::proxy::AccelerometerReading>();
        Eigen::Vector3f v;
        v << ar.getAccelerationX(), ar.getAccelerationY(), 
            ar.getAccelerationZ();
        // Asssumes that all reference axles are the same
        sum += v;
    }
  }
  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

Eigen::Vector3f DroneState::GetMagneticfieldLatest()
{
  odcore::base::Lock l(m_mutex);
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorMagn = m_magnetometerReadings.begin(); 
      sensorMagn != m_magnetometerReadings.end(); ++sensorMagn, ++counter) {
    if (!(*sensorMagn).second.empty()) {
      odcore::data::Container dataContainer = (*sensorMagn).second.back();
      opendlv::proxy::MagnetometerReading mr = 
          dataContainer.getData<opendlv::proxy::MagnetometerReading>();
      Eigen::Vector3f v;
      v << mr.getMagneticFieldX(), mr.getMagneticFieldY(), 
          mr.getMagneticFieldZ();
      // Asssumes that all reference axles are the same
      sum += v;
    }
  }
  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

Eigen::Vector3f DroneState::GetMagneticfieldAvg()
{
  odcore::base::Lock l(m_mutex);
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorMagn = m_magnetometerReadings.begin(); 
      sensorMagn != m_magnetometerReadings.end(); ++sensorMagn) {
    for (std::vector<odcore::data::Container>::iterator dataContainer = 
        (*sensorMagn).second.begin(); 
        dataContainer != (*sensorMagn).second.end(); ++dataContainer, 
        ++counter) {
      opendlv::proxy::MagnetometerReading mr = 
          (*dataContainer).getData<opendlv::proxy::MagnetometerReading>();
        Eigen::Vector3f v;
        v << mr.getMagneticFieldX(), mr.getMagneticFieldY(), 
            mr.getMagneticFieldZ();
        // Asssumes that all reference axles are the same     
        sum += v;
    }
  }

  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

Eigen::Vector3f DroneState::GetAngularVelocityLatest()
{
  odcore::base::Lock l(m_mutex);
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorGyro = m_gyroscopeReadings.begin(); 
      sensorGyro != m_gyroscopeReadings.end(); ++sensorGyro, ++counter) {
    if (!(*sensorGyro).second.empty()) {
      odcore::data::Container dataContainer = (*sensorGyro).second.back();
      opendlv::proxy::GyroscopeReading gr = 
          dataContainer.getData<opendlv::proxy::GyroscopeReading>();
      Eigen::Vector3f v;
      v << gr.getAngularVelocityX(), gr.getAngularVelocityY(), 
          gr.getAngularVelocityZ();
      // Asssumes that all reference axles are the same
      sum += v;
    }
  }
  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

Eigen::Vector3f DroneState::GetAngularVelocityAvg()
{
  odcore::base::Lock l(m_mutex);
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorGyro = m_gyroscopeReadings.begin(); 
      sensorGyro != m_gyroscopeReadings.end(); ++sensorGyro) {
    for (std::vector<odcore::data::Container>::iterator dataContainer = 
        (*sensorGyro).second.begin(); dataContainer != 
        (*sensorGyro).second.end(); ++dataContainer, ++counter) {
      opendlv::proxy::GyroscopeReading gr = 
          (*dataContainer).getData<opendlv::proxy::GyroscopeReading>();
        Eigen::Vector3f v;
        v << gr.getAngularVelocityX(), gr.getAngularVelocityY(), 
            gr.getAngularVelocityZ();
        sum += v;
    }
  }
  

  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

float DroneState::GetTemperatureLatest()
{
  odcore::base::Lock l(m_mutex);
  float sum = 0;
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorTherm = m_temperatureReadings.begin(); 
      sensorTherm != m_temperatureReadings.end(); ++sensorTherm, ++counter) {
    odcore::data::Container dataContainer = (*sensorTherm).second.back();
    opendlv::proxy::TemperatureReading tr = 
        dataContainer.getData<opendlv::proxy::TemperatureReading>();
    sum += static_cast<float>(tr.getTemperature());
  }

  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

float DroneState::GetTemperatureAvg()
{
  odcore::base::Lock l(m_mutex);
  float sum = 0;
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorTherm = m_temperatureReadings.begin(); 
      sensorTherm != m_temperatureReadings.end(); ++sensorTherm) {
    for (std::vector<odcore::data::Container>::iterator dataContainer = 
        (*sensorTherm).second.begin(); dataContainer != 
        (*sensorTherm).second.end(); ++dataContainer, ++counter) {
      opendlv::proxy::TemperatureReading tr = 
          (*dataContainer).getData<opendlv::proxy::TemperatureReading>();
        sum += static_cast<float>(tr.getTemperature());
    }
  }

  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

float DroneState::GetAltitudeLatest()
{
  odcore::base::Lock l(m_mutex);
  float sum = 0;
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorAlt = m_altimeterReadings.begin(); 
      sensorAlt != m_altimeterReadings.end(); ++sensorAlt, ++counter) {
    odcore::data::Container dataContainer = (*sensorAlt).second.back();
    opendlv::proxy::AltimeterReading ar = 
        dataContainer.getData<opendlv::proxy::AltimeterReading>();
    sum += static_cast<float>(ar.getAltitude());
  }

  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

float DroneState::GetAltitudeAvg()
{
  odcore::base::Lock l(m_mutex);
  float sum = 0;
  int counter = 0;
  for (std::map<int32_t, std::vector<odcore::data::Container>>::iterator 
      sensorBar = m_altimeterReadings.begin(); 
      sensorBar != m_altimeterReadings.end(); ++sensorBar) {
    for (std::vector<odcore::data::Container>::iterator dataContainer = 
        (*sensorBar).second.begin(); dataContainer != 
        (*sensorBar).second.end(); ++dataContainer, ++counter) {
      opendlv::proxy::AltimeterReading ar = 
          (*dataContainer).getData<opendlv::proxy::AltimeterReading>();
        sum += static_cast<float>(ar.getAltitude());
    }
  }

  if (counter != 0) {
    return sum / counter;
  } else {
    return sum;
  }
}

}
}
}
