/*
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef COLAA_H
#define COLAA_H

#include <string>
#include <stdint.h>
#include <vector>

#include "lms1xx/colaa_structs.h"

class LMSBuffer;
class MRS1000ScanDataTest;

/**
 * @brief Abstraction for communication with SICK sensors that use
 * CoLa A ASCII Telegram messages.
 *
 * This class implements the protocol for the LMS1xx series of sensors as a baseline.
 *
 * Inherit from this class to implement new sensors that require custom
 * data formats.
 */
class CoLaA
{
friend class MRS1000ScanDataTest;
public:
  CoLaA();
  ~CoLaA();

  /*!
  * @brief Connect to a device supporting the CoLaA protocol.
  * @param host device host name or ip address.
  * @param port device port number.
  */
  void connect(std::string host, int port);

  /*!
  * @brief Disconnect from CoLaA device.
  */
  void disconnect();

  /*!
  * @brief Get status of connection.
  * @returns connected or not.
  */
  bool isConnected() const;

  /*!
  * @brief Log into device
  * Increase privilege level, giving ability to change device configuration.
  */
  void login();

  /*!
  * @brief The device is returned to the measurement mode after configuration.
  *
  */
  void startDevice();

  /*!
  * @brief Start measurements.
  * After receiving this command the unit starts spinning laser and measuring.
  */
  void startMeasurement();

  /*!
  * @brief Stop measurements.
  * After receiving this command LMS1xx unit stop spinning laser and measuring.
  */
  void stopMeasurement();

  /*!
  * @brief Set scan configuration.
  * Set scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @param cfg structure containing scan configuration.
  */
  void setScanConfig(const ScanConfig &cfg);

  /*!
  * @brief Set scan data configuration.
  * Set format of scan message returned by device.
  * @param cfg structure containing scan data configuration.
  */
  void setScanDataConfig(const ScanDataConfig &cfg);

  /**
   * @brief Configures the echo return of the sensor
   * Only supported on LMS5xx and MRS1000
   * @param filter Which echoes to return
   */
  void setEchoFilter(CoLaAEchoFilter::EchoFilter filter);

  void setParticleFilter(bool particle_filter);

  void setMeanFilter(bool mean_filter, uint16_t number_scans);

  /*!
  * @brief Get current scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @returns scanCfg structure.
  */
  ScanConfig getScanConfig();

  /*!
  * @brief Save data permanently.
  * Parameters are saved in the EEPROM of the LMS and will also be available after the device is switched off and on again.
  *
  */
  void saveConfig();

  /*!
  * @brief Get current status of device.
  * This should be the same for all sensor types
  * @returns status of device.
  */
  CoLaAStatus::Status queryStatus();

  /*!
  * @brief Get current output range configuration.
  * Get output range configuration :
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * Should be the same for all sensor types
  * @returns scanOutputRange structure.
  */
  ScanOutputRange getScanOutputRange();

  /*!
  * @brief Start or stop continuous data acquisition.
  * After reception of this command device start or stop continuous data stream containing scan messages.
  * @param start true : start false : stop
  */
  void scanContinuous(bool start);

  /**
   * @brief Requests the last scan output from the device.
   */
  void requestLastScan();

  /*!
  * @brief Receive single scan message.
  * @param scan_data will be passed to parse_scan_data which can be overwritten by subclasses
  * @return true if scan was read successfully, false if error or timeout. False implies that higher level
  *         logic should take correct action such as reopening the connection.
  */
  bool getScanData(void *scan_data);

  /**
   * @brief Query device state
   * @return the device state
   */
  CoLaADeviceState::State getDeviceState();

protected:
  // Command names
  std::string LOGIN_COMMAND;
  std::string LOGIN_USER_MAINT;
  std::string LOGIN_USER_AUTHORIZED;
  std::string LOGIN_USER_SERVICE;
  std::string LOGIN_PASS_MAINT;
  std::string LOGIN_PASS_AUTHORIZED;
  std::string LOGIN_PASS_SERVICE;

  std::string READ_SCAN_CFG_COMMAND;
  std::string SET_SCAN_CFG_COMMAND;
  std::string SET_SCAN_DATA_CFG_COMMAND;
  std::string SET_ECHO_FILTER_COMMAND;
  std::string SET_MEAN_FILTER_COMMAND;
  std::string SET_PARTICLE_FILTER_COMMAND;
  std::string SAVE_CONFIG_COMMAND;

  std::string START_MEASUREMENT_COMMAND;
  std::string STOP_MEASUREMENT_COMMAND;
  std::string REQUEST_SCANS_CONTINUOUSLY;
  std::string REQUEST_LAST_SCAN;

  std::string QUERY_STATUS_COMMAND;

  std::string READ_SCAN_OUTPUT_RANGE_COMMAND;

  std::string START_DEVICE_COMMAND;

  std::string READ_DEVICE_STATE;

  std::string SCAN_DATA_REPLY;

  /**
   * @brief Sends login command
   * @param user_class pick one of LOGIN_USER_x
   * @param password pick matching LOGIN_PASS_x
   */
  void doLogin(std::string user_class, std::string password);

  /**
   * @brief Produce ScanConfig struct from raw ASCII message buffer
   * Can be overwritten by subclasses if customisation is needed.
   * @param buf
   * @param len
   * @return The parsed struct
   */
  virtual ScanConfig parseScanCfg(char *buf, size_t len);

  /**
   * @brief Build up the scan config string from a ScanConfig struct
   * Can be overwritten by subclasses if customisation is needed.
   * @param cfg
   * @return message string
   */
  virtual std::string buildScanCfg(const ScanConfig &cfg) const;

  /**
   * @brief Build scan data config from struct
   * Base implementation will call build_scan_data_cfg_output_channel to construct the
   * output channel part of the message and build_scan_data_cfg_encoder to build the
   * encoder part of the message.
   * @param cfg
   * @return the message string
   */
  virtual std::string buildScanDataCfg(const ScanDataConfig &cfg) const;

  /**
   * @brief Can be customised to implement special message formats for the output channel
   * @param ch Channel number as passed by the ScanDataConfig struct
   * @return output channel part of the message, should not contain leading/trailing spaces
   */
  virtual std::string buildScanDataCfgOutputChannel(int ch) const;

  /**
   * @brief Can be customised to implement special message formats for the encoder
   * @param enc Encoder setting as passed by the ScanDataConfig struct
   * @return encoder part of the message, should not include leading/trailing spaces
   */
  virtual std::string buildScanDataCfgEncoder(int enc) const;

  /**
   * @brief Called by get_scan_data when the internal buffer is filled
   * @param buffer the message to be parsed
   * @param data Destination for the parsed data, pass a ScanData pointer for the base implementation
   */
  virtual bool parseScanData(char *buffer, void *data) const;

  /**
   * @brief Parses the header part of the scan data message and returns it as a struct
   * @param buf the message data
   * @return filled struct
   */
  virtual ScanDataHeader parseScanDataHeader(char **buf) const;

  /**
   * @brief Parses the encoder part of the scan data message
   * The data is currently discarded.
   * @param buf the message data
   */
  virtual void parseScanDataEncoderdata(char **buf) const;

  /**
   * @brief Surrounds command with start and end markers and sends them to the scanner
   * @param command The command string
   */
  void sendCommand(const std::string &command) const;

  /**
   * @brief Surrounds command with start and end markers and sends them to the scanner
   * @param command The command string
   */
  void sendCommand(const char *command) const;

  /**
   * @brief Read data back from the socket
   * Checks for malformed messages and parses error codes.
   * @param buf Destination for the read data
   * @param buflen Maximum size of the buffer. Will be set to new size of the buffer after the read.
   * @return True if the read was successful and no errors occured.
   */
  bool readBack(char *buf, size_t &buflen);

  /**
   * @brief Read data back from the socket and discard it
   * Error checks are still performed.
   * @return
   */
  bool readBack();

private:
  bool connected_;
  LMSBuffer *buffer_;
  int socket_fd_;
};

using LMS1xx = CoLaA; // CoLaA implements the protocol based on the LMS1xx sensor

#endif // COLAA_H
