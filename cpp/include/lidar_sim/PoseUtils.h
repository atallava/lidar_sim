#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    Eigen::MatrixXd getImuTransfFromPose(const std::vector<double> &imu_pose);
    std::vector<double> posnFromImuPose(const std::vector<double> &imu_pose);
}
