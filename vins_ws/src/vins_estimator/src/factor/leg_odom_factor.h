/*******************************************************
 * Leg odometry relative factor for dog_vins_localization.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cmath>

class LegOdomFactor
{
  public:
    // mode 0: distance + yaw, robust to unknown camera/base planar axes.
    // mode 1: local dx + dy + yaw, use only after body/base alignment is known.
    LegOdomFactor(double dx, double dy, double dyaw,
                  double pos_std, double yaw_std, int mode);

    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const
    {
        Eigen::Matrix<T, 3, 1> Pi(pose_i[0], pose_i[1], pose_i[2]);
        Eigen::Quaternion<T> Qi(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        Eigen::Matrix<T, 3, 1> Pj(pose_j[0], pose_j[1], pose_j[2]);
        Eigen::Quaternion<T> Qj(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);

        Qi.normalize();
        Qj.normalize();
        Eigen::Matrix<T, 3, 1> dp = Qi.conjugate() * (Pj - Pi);
        Eigen::Quaternion<T> dq = Qi.conjugate() * Qj;
        dq.normalize();

        using std::atan2;
        using std::sqrt;
        const T pred_yaw = atan2(T(2.0) * (dq.w() * dq.z() + dq.x() * dq.y()),
                                 T(1.0) - T(2.0) * (dq.y() * dq.y() + dq.z() * dq.z()));
        const T yaw_residual = normalizeAngle(pred_yaw - T(meas_dyaw_));

        if (mode_ == 1)
        {
            residuals[0] = (dp.x() - T(meas_dx_)) * T(sqrt_info_pos_);
            residuals[1] = (dp.y() - T(meas_dy_)) * T(sqrt_info_pos_);
            residuals[2] = yaw_residual * T(sqrt_info_yaw_);
        }
        else
        {
            const T pred_dist = sqrt(dp.squaredNorm() + T(1e-12));
            const T meas_dist = T(meas_distance_);
            residuals[0] = (pred_dist - meas_dist) * T(sqrt_info_pos_);
            residuals[1] = T(0.0);
            residuals[2] = yaw_residual * T(sqrt_info_yaw_);
        }
        return true;
    }

    static ceres::CostFunction* Create(double dx, double dy, double dyaw,
                                       double pos_std, double yaw_std, int mode)
    {
        return new ceres::AutoDiffCostFunction<LegOdomFactor, 3, 7, 7>(
            new LegOdomFactor(dx, dy, dyaw, pos_std, yaw_std, mode));
    }

  private:
    template <typename T>
    static T normalizeAngle(const T& angle)
    {
        using std::atan2;
        using std::cos;
        using std::sin;
        return atan2(sin(angle), cos(angle));
    }

    double meas_dx_;
    double meas_dy_;
    double meas_dyaw_;
    double meas_distance_;
    double sqrt_info_pos_;
    double sqrt_info_yaw_;
    int mode_;
};
