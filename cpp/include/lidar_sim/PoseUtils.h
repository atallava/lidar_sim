#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    Eigen::MatrixXd getImuTransfFromPose(std::vector<double> imu_pose);
}
