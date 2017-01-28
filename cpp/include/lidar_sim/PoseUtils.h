#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    Eigen::MatrixXd getImuTransfFromPose(std::vector<double> imu_pose);
    std::vector<std::vector<double> > genRayDirnsWorldFrame(std::vector<double> imu_pose, LaserCalibParams laser_calib_params);
    std::vector<std::vector<double> > genRayDirnsLaserFrame(LaserIntrinsics intrinsics);
    std::vector<std::vector<double> > rotateRayDirns(Eigen::MatrixXd R, std::vector<std::vector<double> > ray_dirns);
}
