#pragma once

#include "controller_and_estimator.h"
#include <stdint.h>
#include <sys/cdefs.h>

struct __packed EntryFlags {
  // these are mutually exclusive except for imu and gps
  // if there is both imu and gps packet, imu packet comes first
  bool new_imu_packet : 1;
  bool new_gps_packet : 1;
  bool controller_state : 1;
  bool calib : 1;
  bool controller_out : 1;
  bool entry : 1;
};

static_assert(sizeof(EntryFlags) == 1, "sizeof(EntryFlags) error");

struct __packed EntryBase {
  float time;
  uint8_t phase;
};

static_assert(sizeof(EntryBase) == 5, "sizeof(EntryBase) error");

struct __packed SensorEntry {

  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;

  float mag_x;
  float mag_y;
  float mag_z;
};

static_assert(sizeof(SensorEntry) == 36, "sizeof(SensorEntry) error");

struct __packed GpsEntry {

  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;

  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float posCovNN; // m^2
  float posCovNE; // m^2
  float posCovND; // m^2
  float posCovEE; // m^2
  float posCovED; // m^2
  float posCovDD; // m^2
  float velCovNN; // m^2/s^2
  float velCovNE; // m^2/s^2
  float velCovND; // m^2/s^2
  float velCovEE; // m^2/s^2
  float velCovED; // m^2/s^2
  float velCovDD; // m^2/s^2
};

static_assert(sizeof(GpsEntry) == 18 * 4, "sizeof(GpsEntry) error");

namespace TrajectoryLogger {

void log_trajectory_flash(float time, int phase, const Controller_Input ci, const Controller_Output co);
void log_calib_flash();

}; // namespace TrajectoryLogger
