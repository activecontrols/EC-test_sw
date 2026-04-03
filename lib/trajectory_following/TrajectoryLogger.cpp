#include "TrajectoryLogger.h"
#include "FlashLogging.h"
#include "GPS.h"
#include "Router.h"
#include "SDCard.h"

#include "flash.h"

namespace TrajectoryLogger {

File positionLogFile;
CString<400> telemCSV;

#define LOG_HEADER ("time,phase,north,west,up")

// // logs time, phase, and position data in .csv format
// int print_counter = 0;
// void log_trajectory_csv(float time, int phase, Controller_Input ci, Controller_Output co) {
//   telemCSV.clear();
//   telemCSV << time << "," << phase << "," << ci.target_pos_north << "," << ci.target_pos_west << "," << ci.target_pos_up;

//   positionLogFile.println(telemCSV.str);
//   positionLogFile.flush();

//   print_counter++;
//   if (print_counter % 10 == 0) {
//     telemCSV.clear();
//     telemCSV << time << "  " << ci.target_pos_north << "  " << ci.target_pos_west << "  " << ci.target_pos_up;
//     Router::print(telemCSV.str);
//   }
// }

struct __packed EntryFlags {
  // these are mutually exclusive except for imu and gps
  // if there is both imu and gps packet, imu packet comes first
  bool new_imu_packet : 1;
  bool new_gps_packet : 1;
  bool x_est : 1;
  bool calib : 1;
  bool controller_out : 1;
  bool entry : 1;
  bool flight_p : 1;
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

void log_x_est() {
  EntryFlags flags{};

  flags.x_est = 1;

  Logging::write((uint8_t *)&flags, sizeof(flags));

  // log x_est
  Logging::write((uint8_t *)(ControllerAndEstimator::x_est.data()), sizeof(ControllerAndEstimator::x_est(0)) * (ControllerAndEstimator::x_est.size()));

  return;
}

void log_flight_p() {
  EntryFlags flags{};

  flags.flight_p = 1;

  Logging::write((uint8_t *)&flags, sizeof(flags));

  Logging::write((uint8_t *)(ControllerAndEstimator::Flight_P.data()), sizeof(ControllerAndEstimator::Flight_P(0)) * (ControllerAndEstimator::Flight_P.size()));

  return;
}

void log_trajectory_flash(float time, int phase, Controller_Input ci, Controller_Output co) {

  EntryFlags flags{};
  flags.new_gps_packet = ci.new_gps_packet;
  flags.new_imu_packet = ci.new_imu_packet;
  flags.controller_out = 1;
  flags.entry = 1;

  Logging::write((uint8_t *)&flags, sizeof(flags));

  EntryBase entryBase{};
  entryBase.time = time;
  entryBase.phase = phase;

  Logging::write((uint8_t *)&entryBase, sizeof(entryBase));

  if (ci.new_imu_packet) {
    SensorEntry sensorData{};
    sensorData.accel_x = ci.accel_x;
    sensorData.accel_y = ci.accel_y;
    sensorData.accel_z = ci.accel_z;
    sensorData.gyro_yaw = ci.gyro_yaw;
    sensorData.gyro_pitch = ci.gyro_pitch;
    sensorData.gyro_roll = ci.gyro_roll;
    sensorData.mag_x = ci.mag_x;
    sensorData.mag_y = ci.mag_y;
    sensorData.mag_z = ci.mag_z;

    Logging::write((uint8_t *)&sensorData, sizeof(sensorData));
  }

  if (ci.new_gps_packet) {
    GpsEntry gpsData{};
    gpsData.gps_pos_north = ci.gps_pos_north;
    gpsData.gps_pos_west = ci.gps_pos_west;
    gpsData.gps_pos_up = ci.gps_pos_up;

    gpsData.gps_vel_north = ci.gps_vel_north;
    gpsData.gps_vel_west = ci.gps_vel_west;
    gpsData.gps_vel_up = ci.gps_vel_up;

    gpsData.posCovNN = GPS::ubx.cov.data->posCovNN;
    gpsData.posCovNE = GPS::ubx.cov.data->posCovNE;
    gpsData.posCovND = GPS::ubx.cov.data->posCovND;
    gpsData.posCovEE = GPS::ubx.cov.data->posCovEE;
    gpsData.posCovED = GPS::ubx.cov.data->posCovED;
    gpsData.posCovDD = GPS::ubx.cov.data->posCovDD;

    gpsData.velCovNN = GPS::ubx.cov.data->velCovNN;
    gpsData.velCovNE = GPS::ubx.cov.data->velCovNE;
    gpsData.velCovND = GPS::ubx.cov.data->velCovND;
    gpsData.velCovEE = GPS::ubx.cov.data->velCovEE;
    gpsData.velCovED = GPS::ubx.cov.data->velCovED;
    gpsData.velCovDD = GPS::ubx.cov.data->velCovDD;

    Logging::write((uint8_t *)&gpsData, sizeof(gpsData));
  }

  static_assert(sizeof(co) == 16);
  // log controller output
  Logging::write((uint8_t *)&co, sizeof(co));

  return;
}

// write IMU calib, MAG calib to flash
void log_calib_flash() {

  EntryFlags flags{};

  flags.calib = 1;

  Logging::write((uint8_t *)&flags, sizeof(flags));
  // some checks to make sure the compiler isn't adding weird padding
  static_assert(sizeof(IMU::Calib) == 8 * 9);
  static_assert(sizeof(Mag::calibration) == 12 * 8);
  static_assert(IMU_COUNT == 1);

  // write imu calibration to flash (we are only using IMU 0 for now)
  Logging::write((uint8_t *)&IMU::IMUs[0].calib, sizeof(IMU::IMUs[0].calib));

  // write mag calibration to flash
  Logging::write((uint8_t *)&Mag::calib, sizeof(Mag::calib));

  return;
}

void log_complete() {
  Logging::complete();
}

void flash_dump_test() {
  uint8_t last_page[PAGE_SIZE];
  uint32_t addr = 0;
  int idx = 0;
  while (1) {
    // wait for serial to send a c before sending the next page
    while (!Serial.available()) {
    }

    char c;

    do {
      c = Serial.read();
    } while (!(c == 'k' || c == 'c'));

    if (c == 'k') {
      return;
    }

    Flash::read(addr, PAGE_SIZE, last_page);

    Serial.write(last_page, sizeof(last_page));

    bool end = true;
    for (int i = 0; i < sizeof(last_page); ++i) {
      if (last_page[i] != 0xFF) {
        end = false;
        break;
      }
    }

    if (end) {
      Serial.write('k');
      break;
    }

    Serial.write('c');

    idx %= PAGE_SIZE;

    addr += PAGE_SIZE;
  }

  return;
}

// creates a log file for the current trajectory and prints csv header
// void create_trajectory_log(const char *filename) {
//   positionLogFile = SDCard::open(filename, FILE_WRITE);
//   positionLogFile.println(LOG_HEADER);
// }

// // close and flush the log file
// void close_trajectory_log() {
//   positionLogFile.flush();
//   positionLogFile.close();
// }

} // namespace TrajectoryLogger