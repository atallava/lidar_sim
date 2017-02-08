#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    class PoseServer {
    public:
	PoseServer(std::string rel_path_pose_log);
	void printLogAtIndex(int data_id);
	void printPose(const std::vector<double> pose);
	std::vector<double> getPoseAtTime (double t) const;
	Eigen::Matrix<float,4,4> getTransfAtTime(double t);
	std::tuple<std::vector<double>, int> 
	    getNearestPoseInLog(const std::vector<double> pose);

    private:
	void m_loadPoseLog(std::string rel_path_pose_log);
	
	std::vector<std::vector<double>> m_pose_log;
	std::vector<std::vector<double> > m_posn_log;
	std::vector<double> m_t_log;
	std::string m_path_pose_log;
	int m_num_logs;
    };
}

