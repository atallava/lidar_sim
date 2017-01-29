#pragma once
#include <vector>
#include <string>
#include <tuple>

#include <Eigen/Dense>

#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    std::vector<std::vector<double> > genRayDirnsWorldFrame(std::vector<double> imu_pose, LaserCalibParams laser_calib_params);
    std::vector<std::vector<double> > genRayDirnsLaserFrame(LaserIntrinsics intrinsics);
    std::vector<std::vector<double> > rotateRayDirns(Eigen::MatrixXd R, std::vector<std::vector<double> > ray_dirns);
    std::tuple<std::vector<double>, double> calcRayDirn(std::vector<double> ray_origin, std::vector<double> end_pt);
}
