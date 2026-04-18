#pragma once

#include <Arduino.h>

namespace Mag {

struct calibration {
  float hard_x;
  float hard_y;
  float hard_z;
  float soft[3][3];
};

extern const char *main_calib_name; // defined in mag.cpp as "mag_calib.bin"

void begin();
bool read_xyz_calibrated(float &mx, float &my, float &mz);
float get_heading();

bool isMeasurementReady();
bool beginMeasurement();

extern calibration calib;
} // namespace Mag