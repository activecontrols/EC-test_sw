// Patch file to allow running embedded controller without modifying embedded code
#include <Eigen/Dense>
#define max(a, b) (((a) > (b)) ? (a) : (b))
