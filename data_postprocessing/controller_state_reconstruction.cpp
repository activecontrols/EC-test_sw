#define __packed __attribute__((__packed__)) // patch this in
#include "Trajectory.h"
#include "TrajectoryLogger.h"
#include "controller_and_estimator.h"
#include "flight_data.h"
#include <stdio.h>
#include <stdlib.h>

flight_packet_t fp; // allows this to persist across calls

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

bool parse_log_entry(FILE *compressed_bin, FILE *reconstructed_bin) {
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
    fp.GND_flag = false;
    fp.flight_armed = true;

    // TODO - fill these
    // fp.thrust_perc;
    // fp.diffy_perc;
    // fp.rtk_status;

    EntryBase eb;
    fread(&eb, sizeof(EntryBase), 1, compressed_bin);
    float dT = eb.time - last_time;
    last_time = eb.time;
    fp.elapsed_time = eb.time;

    ci.target_pos_north = trajectory[eb.phase].north;
    ci.target_pos_west = trajectory[eb.phase].west;
    ci.target_pos_up = trajectory[eb.phase].up;
    fp.target_pos_north = ci.target_pos_north;
    fp.target_pos_west = ci.target_pos_west;
    fp.target_pos_up = ci.target_pos_up;

    if (flags.new_imu_packet) { // TODO - make sure we are handling value persistence the correct way here and in the flight loop
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

      fp.accel_x = ci.accel_x;
      fp.accel_y = ci.accel_y;
      fp.accel_z = ci.accel_z;
      fp.gyro_yaw = ci.gyro_yaw;
      fp.gyro_pitch = ci.gyro_pitch;
      fp.gyro_roll = ci.gyro_roll;
      fp.mag_x = ci.mag_x;
      fp.mag_y = ci.mag_y;
      fp.mag_z = ci.mag_z;
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

      fp.gps_pos_north = ci.gps_pos_north;
      fp.gps_pos_west = ci.gps_pos_west;
      fp.gps_pos_up = ci.gps_pos_up;
      fp.gps_vel_north = ci.gps_vel_north;
      fp.gps_vel_west = ci.gps_vel_west;
      fp.gps_vel_up = ci.gps_vel_up;
    } else {
      ci.new_gps_packet = false;
    }

    if (flags.controller_out) {
      Controller_Output logged_co;
      fread(&logged_co, sizeof(Controller_Output), 1, compressed_bin);
      printf("%f %f %f %f - ", logged_co.thrust_N, logged_co.roll_rad_sec_squared, logged_co.gimbal_pitch_deg, logged_co.gimbal_yaw_deg);

      fp.gimbal_yaw_raw = logged_co.gimbal_yaw_deg;
      fp.gimbal_pitch_raw = logged_co.gimbal_pitch_deg;
      fp.thrust_N = logged_co.thrust_N;
      fp.roll_rad_sec_squared = logged_co.roll_rad_sec_squared;
    }

    Controller_State cs;
    Controller_Output constructed_co = ControllerAndEstimator::get_controller_output(ci, dT, &cs);
    printf("%f %f %f %f\n", constructed_co.thrust_N, constructed_co.roll_rad_sec_squared, constructed_co.gimbal_pitch_deg, constructed_co.gimbal_yaw_deg);

    fp.state_q_vec_new = cs.state_q_vec_new;
    fp.state_q_vec_0 = cs.state_q_vec_0;
    fp.state_q_vec_1 = cs.state_q_vec_1;
    fp.state_q_vec_2 = cs.state_q_vec_2;
    fp.state_pos_north = cs.state_pos_north;
    fp.state_pos_west = cs.state_pos_west;
    fp.state_pos_up = cs.state_pos_up;
    fp.state_vel_north = cs.state_vel_north;
    fp.state_vel_west = cs.state_vel_west;
    fp.state_vel_up = cs.state_vel_up;
    fp.gyro_bias_yaw = cs.gyro_bias_yaw;
    fp.gyro_bias_pitch = cs.gyro_bias_pitch;
    fp.gyro_bias_roll = cs.gyro_bias_roll;
    fp.accel_bias_x = cs.accel_bias_x;
    fp.accel_bias_y = cs.accel_bias_y;
    fp.accel_bias_z = cs.accel_bias_z;
    fp.mag_bias_x = cs.mag_bias_x;
    fp.mag_bias_y = cs.mag_bias_y;
    fp.mag_bias_z = cs.mag_bias_z;

    fwrite(&fp, sizeof(fp), 1, reconstructed_bin);
  }

  return true;
}

int main() {
  FILE *compressed_bin = fopen("flash_dump.bin", "rb");
  if (compressed_bin == NULL) {
    printf("Failed to open compressed log dump.");
    return EXIT_FAILURE;
  }

  FILE *reconstructed_bin = fopen("reconstructed_flight.bin", "wb");
  if (reconstructed_bin == NULL) {
    fclose(compressed_bin);
    printf("Failed to create reconstructed flight log.");
    return EXIT_FAILURE;
  }

  last_time = 0;
  ControllerAndEstimator::init_controller_and_estimator_constants();
  while (parse_log_entry(compressed_bin, reconstructed_bin)) {
  };

  printf("Finished reading log.");
  fclose(compressed_bin);
  fclose(reconstructed_bin);
  return EXIT_SUCCESS;
}