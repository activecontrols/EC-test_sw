#include "Mag_Calib.h"
#include "CommsSerial.h"
#include "matlab_funcs.h"

using Vector1000 = Eigen::Matrix<float, CALIB_SAMPLE_COUNT, 1>;
using Matrix1000x9 = Eigen::Matrix<float, CALIB_SAMPLE_COUNT, 9>;

Mag::calibration MagCalib(float read_x[CALIB_SAMPLE_COUNT], float read_y[CALIB_SAMPLE_COUNT], float read_z[CALIB_SAMPLE_COUNT]) {
  // Script uses least squares to fit an ellipsoid model

  Eigen::Map<Vector1000> X(read_x);
  Eigen::Map<Vector1000> Y(read_y);
  Eigen::Map<Vector1000> Z(read_z);

  Matrix1000x9 D = (Matrix1000x9() << X.cwiseAbs2(), Y.cwiseAbs2(), Z.cwiseAbs2(), 2 * X.cwiseProduct(Y), 2 * X.cwiseProduct(Z), 2 * Y.cwiseProduct(Z), X, Y, Z).finished();

  // Set up least squares:
  Vector9 P = (D.transpose() * D).completeOrthogonalDecomposition().pseudoInverse() * D.transpose() * Vector1000::Ones();

  // Reconstruct parameters
  Matrix3_3 A = (Matrix3_3() << P(0), P(3), P(4), P(3), P(1), P(5), P(4), P(5), P(2)).finished();
  Vector3 B = (Vector3() << P[6], P[7], P[8]).finished();

  // Reconstruct hard iron
  Vector3 HI = (-1 / 2.0) * A.inverse() * B;

  // Reconstruct soft iron
  float k = 1 + HI.transpose() * A * HI;
  Matrix3_3 A_scaled = A / k;

  // Use eigenvalue decomposition to find the matrix square root.
  Eigen::EigenSolver<Matrix3_3> eigensolver;
  eigensolver.compute(A_scaled);
  Matrix3_3 E = eigensolver.eigenvalues().real().asDiagonal();
  Matrix3_3 R = eigensolver.eigenvectors().real();

  // Since E is a diagonal matrix of eigenvalues, its square root
  // is just the square root of its diagonal elements.
  Matrix3_3 SI = R * E.cwiseSqrt() * R.transpose();

  Mag::calibration calib;
  calib.hard_x = HI(0);
  calib.hard_y = HI(1);
  calib.hard_z = HI(2);
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      calib.soft[r][c] = SI(r, c);
    }
  }

  // VERIFY CALIBRATION
  float mags_raw[CALIB_SAMPLE_COUNT];
  float mags_calib[CALIB_SAMPLE_COUNT];
  float SS_res = 0;
  float SS_tot = 0;
  float mags_raw_sum = 0;

  for (int i = 0; i < CALIB_SAMPLE_COUNT; i++) {
    mags_raw[i] = sqrt(pow(read_x[i], 2) + pow(read_y[i], 2) + pow(read_z[i], 2));
    mags_raw_sum += mags_raw[i];

    float x = read_x[i] - calib.hard_x;
    float y = read_y[i] - calib.hard_y;
    float z = read_z[i] - calib.hard_z;

    float cal_x = calib.soft[0][0] * x + calib.soft[0][1] * y + calib.soft[0][2] * z;
    float cal_y = calib.soft[1][0] * x + calib.soft[1][1] * y + calib.soft[1][2] * z;
    float cal_z = calib.soft[2][0] * x + calib.soft[2][1] * y + calib.soft[2][2] * z;

    mags_calib[i] = sqrt(pow(cal_x, 2) + pow(cal_y, 2) + pow(cal_z, 2));
    SS_res += pow((mags_calib[i] - 1), 2);
  }
  float mags_raw_avg = mags_raw_sum / CALIB_SAMPLE_COUNT;

  for (int i = 0; i < CALIB_SAMPLE_COUNT; i++) {
    SS_tot += pow((mags_raw[i] - mags_raw_avg), 2);
  }

  float R_square = 1 - SS_res / SS_tot;
  CommsSerial.printf("Calibration Sphere Fit R Squared: %f\n", R_square);

  return calib;
}
