#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/PoseUtils.h>

namespace lidar_sim {
    Eigen::MatrixXd getImuTransfFromPose(std::vector<double> imu_pose)
    {
	double y = imu_pose[0];
	double x = imu_pose[1];
	double z = imu_pose[2];
	double roll = imu_pose[3];
	double pitch = imu_pose[4];
	double yaw = imu_pose[5];

	Eigen::MatrixXd T_pose(4,4);

	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d rotationMatrix = (yawAngle*pitchAngle*rollAngle).toRotationMatrix();

	for (size_t i = 0; i < 3; ++i)
	    for (size_t j = 0; j < 3; ++j)
		T_pose(i,j) = rotationMatrix(i,j);
	T_pose(0,3) = x; T_pose(1,3) = y; T_pose(2,3) = z;
	T_pose(3,0) = 0; T_pose(3,1) = 0; T_pose(3,2) = 0; T_pose(3,3) = 1;

	return T_pose;
    }
}
