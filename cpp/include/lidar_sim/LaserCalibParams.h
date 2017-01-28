#pragma once

#include <Eigen/Dense>

namespace lidar_sim {
    struct LaserIntrinsics {
	LaserIntrinsics();
	std::vector<double> alpha_vec;
	std::vector<double> theta_vec;
	double min_range;
	double max_range;
    };

    struct LaserExtrinsics {
    	LaserExtrinsics();
    	Eigen::MatrixXd T_laser_imu;
    };
    
    struct LaserCalibParams {
    	LaserCalibParams();
    	LaserIntrinsics intrinsics;
    	LaserExtrinsics extrinsics;
    };
}
