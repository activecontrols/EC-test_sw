#define __packed __attribute__((__packed__)) // patch this in
#include "Trajectory.h"
#include "TrajectoryLogger.h"
#include "controller_and_estimator.h"
#include <stdio.h>
#include <stdlib.h>

traj_point_pos trajectory[3] = {{0, 0, 0, 0}, {60, 0, 0, 3}, {120, 0, 0, 0}}; // TODO - having this in the log file would be nice

struct IMU_Calib {
  double gyro_bias[3];
  double accel_correction_bias[3];
  double accel_correction_gain[3];
};

struct MAG_Calib {
  double hard_x;
  double hard_y;
  double hard_z;
  double soft[9];
};

float last_time;

bool parse_log_entry(FILE *compressed_bin) {
  EntryFlags flags;
  size_t did_read = fread(&flags, sizeof(EntryFlags), 1, compressed_bin);
  if (did_read != sizeof(EntryFlags)) {
    return false;
  }

  printf("Read log entry.");
  if (flags.calib) {
    IMU_Calib i_calib;
    MAG_Calib m_calib;
    fread(&i_calib, sizeof(IMU_Calib), 1, compressed_bin);
    fread(&m_calib, sizeof(MAG_Calib), 1, compressed_bin);
  }
  if (flags.controller_state) { // TODO - this doesn't work right now
    float controller_state[100];
    fread(controller_state, sizeof(float), 100, compressed_bin);
  }

  // normal packet
  if (flags.entry) {
    Controller_Input ci;
    ci.GND_val = false;

    EntryBase eb;
    fread(&eb, sizeof(EntryBase), 1, compressed_bin);
    float dT = eb.time - last_time;
    last_time = eb.time;
    ci.target_pos_north = trajectory[eb.phase].north;
    ci.target_pos_west = trajectory[eb.phase].west;
    ci.target_pos_up = trajectory[eb.phase].up;

    if (flags.new_imu_packet) {
      SensorEntry se;
      fread(&se, sizeof(SensorEntry), 1, compressed_bin);

      ci.new_imu_packet = true;
      ci.accel_x = se.accel_x;
      ci.accel_y = se.accel_y;
      ci.accel_z = se.accel_z;
      ci.gyro_yaw = se.gyro_yaw;
      ci.gyro_pitch = se.gyro_pitch;
      ci.gyro_roll = se.gyro_roll;
      ci.mag_x = se.mag_x;
      ci.mag_y = se.mag_y;
      ci.mag_z = se.mag_z;
    } else {
      ci.new_imu_packet = false;
    }

    if (flags.new_gps_packet) {
      GpsEntry gps;
      fread(&gps, sizeof(GpsEntry), 1, compressed_bin);

      ci.new_gps_packet = true;
      ci.gps_pos_north = gps.gps_pos_north;
      ci.gps_pos_west = gps.gps_pos_west;
      ci.gps_pos_up = gps.gps_pos_up;
      ci.gps_vel_north = gps.gps_vel_north;
      ci.gps_vel_west = gps.gps_vel_west;
      ci.gps_vel_up = gps.gps_vel_up;
    } else {
      ci.new_gps_packet = false;
    }

    if (flags.controller_out) {
      Controller_Output logged_co;
      fread(&logged_co, sizeof(Controller_Output), 1, compressed_bin);
      printf("%f %f %f %f - ", logged_co.thrust_N, logged_co.roll_rad_sec_squared, logged_co.gimbal_pitch_deg, logged_co.gimbal_yaw_deg);
    }

    Controller_Output constructed_co = ControllerAndEstimator::get_controller_output(ci, dT);
    printf("%f %f %f %f\n", constructed_co.thrust_N, constructed_co.roll_rad_sec_squared, constructed_co.gimbal_pitch_deg, constructed_co.gimbal_yaw_deg);
  }

  return true;
}

int main() {
  FILE *compressed_bin = fopen("flash_dump.bin", "rb");
  if (compressed_bin == NULL) {
    printf("Failed to open compressed log dump.");
    return EXIT_FAILURE;
  }

  last_time = 0;
  ControllerAndEstimator::init_controller_and_estimator_constants();
  while (parse_log_entry(compressed_bin)) {
  };

  printf("Finished reading log.");
  return EXIT_SUCCESS;
}