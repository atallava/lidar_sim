#pragma once
#include <vector>
#include <string>
#include <tuple>

#include <Eigen/Dense>

#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    std::vector<std::vector<double> > genRayDirnsWorldFrame(const std::vector<double> &imu_pose, const LaserCalibParams laser_calib_params);
    std::vector<std::vector<double> > genRayDirnsLaserFrame(const LaserIntrinsics intrinsics);
    std::vector<std::vector<double> > rotateRayDirns(const Eigen::MatrixXd &R, const std::vector<std::vector<double> > &ray_dirns);
    std::vector<double> laserPosnFromImuPose(const std::vector<double> &imu_pose, const LaserCalibParams laser_calib_params);
}
