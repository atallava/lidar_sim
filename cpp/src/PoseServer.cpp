#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <Eigen/Dense>

#include <lidar_sim/PoseServer.h>

using namespace lidar_sim;

PoseServer::PoseServer(std::string rel_path_pose_log) :
    m_path_pose_log(rel_path_pose_log),
    m_num_logs(0)
{
    m_loadPoseLog(m_path_pose_log);
}

void PoseServer::m_loadPoseLog(std::string path_pose_log)
{
    std::ifstream pose_log_file(path_pose_log);
    std::string current_line;
    m_num_logs = 0;
    while(std::getline(pose_log_file, current_line))
    {
	m_num_logs++;
	double data;
	std::istringstream iss(current_line);
	iss >> data;    

	// debug
	// std::cout << "data id: " << m_num_logs << std::endl;
	// std::cout << "line: " << current_line << std::endl;
	// std::cout << std::setprecision(20) << "t: " << data << std::endl;

	m_t_log.push_back(data);

	// debug
	// std::cout << "pose : ";

	std::vector<double> xyzrpy;
	for (size_t i = 0; i < 6; ++i)
	{
	    iss >> data;
	    xyzrpy.push_back(data);

	    // debug
	    // std::cout << data << " ";
	}
	// std::cout << std::endl;

	m_pose_log.push_back(xyzrpy);		      

	// if (m_num_logs > 10)
	//     break;
    }
    pose_log_file.close();
    std::cout << "Loaded imu poses from: " << path_pose_log << std::endl;
}

void PoseServer::printLogAtIndex(int data_id)
{
    std::cout << "id: " << data_id << std::endl;
    std::cout << std::setprecision(20) << "t: " << m_t_log[data_id] << std::endl;
    printPose(m_pose_log[data_id]);
}

void PoseServer::printPose(std::vector<double> pose)
{
   std::cout << "pose: ";
    for(size_t i = 0; i < 6; i++)
	std::cout << std::setprecision(5) << pose[i] << " ";
    std::cout << std::endl;
}

std::vector<double> PoseServer::getPoseAtTime(double t)
{
    auto it = std::lower_bound(m_t_log.begin(), m_t_log.end(), t);
    size_t index_2 = std::distance(m_t_log.begin(), it);

    // t greater than max time
    if (index_2 == m_t_log.size())
    {
	std::cout << t << " outside range, greater than max time" << std::endl;
	throw std::range_error("query time outside range");
    }

    // exact match
    if (m_t_log[index_2] == t)
	return m_pose_log[index_2];

    // t less than min time
    if (index_2 == 0)
    {
	std::cout << t << " outside range, less than min time" << std::endl;
	throw std::range_error("query time outside range");
    }

    size_t index_1 = index_2-1;

    std::vector<double> pose(6, 0.0);
    double alpha = (t-m_t_log[index_1])/(m_t_log[index_2]-m_t_log[index_1]);
    for (size_t i = 0; i < 6; i++)
	pose[i] = m_pose_log[index_1][i] + alpha*(m_pose_log[index_2][i] - m_pose_log[index_1][i]);

    return pose;
}

Eigen::Matrix<float,4,4> PoseServer::getTransfAtTime(double t)
{
    std::vector<double> pose = getPoseAtTime(t);
    double x = pose[0];
    double y = pose[1];
    double z = pose[2];
    double roll = pose[3];
    double pitch = pose[4];
    double yaw = pose[5];

    Eigen::Matrix<float,4,4> T_pose;

    T_pose << 
	cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll), x,
	sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll), y,
	-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), z,
	0, 0, 0, 1;

    return T_pose;
}
