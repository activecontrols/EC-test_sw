#pragma once

#include "Mag.h"
#define CALIB_SAMPLE_COUNT 1000

Mag::calibration MagCalib(float read_x[CALIB_SAMPLE_COUNT], float read_y[CALIB_SAMPLE_COUNT], float read_z[CALIB_SAMPLE_COUNT]);
