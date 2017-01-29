#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    Eigen::MatrixXd getImuTransfFromPose(std::vector<double> imu_pose);
    std::vector<double> posnFromImuPose(std::vector<double> imu_pose);
}
