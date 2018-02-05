#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    class PoseServer {
    public:
	PoseServer();
	PoseServer(std::string rel_path_pose_log);
	void loadPoseLog(std::string rel_path_pose_log);
	void printLogAtIndex(int data_id);
	void printPose(const std::vector<double> pose);
	std::vector<double> getPoseAtTime (double t) const;
	Eigen::Matrix<double,4,4> getTransfAtTime(double t);
	std::tuple<std::vector<double>, int> 
	    getNearestPoseInLog(const std::vector<double> pose);
	double getTimeAtIndex(size_t idx);
	// largest index s.t. m_t_log[idx] is < t
	size_t getIndexOfNearestTime(double t);
	size_t getNumLogs();

    private:
	std::vector<std::vector<double>> m_pose_log;
	std::vector<std::vector<double> > m_posn_log;
	std::vector<double> m_t_log;
	std::string m_path_pose_log;
	int m_num_logs;
    };
}

