#include "leg_odom_factor.h"

#include <algorithm>
#include <cmath>

LegOdomFactor::LegOdomFactor(double dx, double dy, double dyaw,
                             double pos_std, double yaw_std, int mode)
    : meas_dx_(dx),
      meas_dy_(dy),
      meas_dyaw_(dyaw),
      meas_distance_(std::sqrt(dx * dx + dy * dy)),
      mode_(mode)
{
    pos_std = std::max(pos_std, 1e-4);
    yaw_std = std::max(yaw_std, 1e-4);
    sqrt_info_pos_ = 1.0 / pos_std;
    sqrt_info_yaw_ = 1.0 / yaw_std;
}
