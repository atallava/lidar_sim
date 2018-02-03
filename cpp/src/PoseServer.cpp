#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

PoseServer::PoseServer() :
    m_num_logs(0)
{
    // todo: something here says that not ready for use
}

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
    std::vector<double> posn0(3,0);
	
    while(std::getline(pose_log_file, current_line))
    {
	m_num_logs++;
	double data;
	std::istringstream iss(current_line);
	iss >> data;    // 30 sec delay timestamp

	iss >> data; // timestamp we want
	m_t_log.push_back(data);

	// ignore lat and lon
	for (size_t i = 0; i < 2; ++i)
	    iss >> data; 

	std::vector<double> pose;
	// yxz
	for (size_t i = 0; i < 3; ++i)
	{
	    iss >> data;
	    pose.push_back(data);
	}

	// bunch of values we don't need
	for (size_t i = 0; i < 19; ++i)
	    iss >> data;

	// rpy
	for (size_t i = 0; i < 3; ++i)
	{
	    iss >> data;
	    // convert angles to radian
	    pose.push_back(data*M_PI/180);
	}

	// if first time, get posn0
	if (m_num_logs == 1)
	    for (size_t i = 0; i < 3; ++i)
		posn0[i] = pose[i];

	// subtract posn0
	for (size_t i = 0; i < 3; ++i)
	    pose[i] = pose[i]-posn0[i];

	// debug
	// std::cout << "pose : ";

	m_pose_log.push_back(pose);		      

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

std::vector<double> PoseServer::getPoseAtTime (double t) const
{
    auto it = std::lower_bound(m_t_log.begin(), m_t_log.end(), t);
    size_t index_2 = std::distance(m_t_log.begin(), it);

    // t greater than max time
    if (index_2 == m_t_log.size())
    {
	std::cout << std::setprecision(20) << t << " outside range, greater than max time" << std::endl;
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

    // linear interpolation
    std::vector<double> pose(6, 0.0);
    double alpha = (t-m_t_log[index_1])/(m_t_log[index_2]-m_t_log[index_1]);
    for (size_t i = 0; i < 6; i++)
	pose[i] = m_pose_log[index_1][i] + alpha*(m_pose_log[index_2][i] - m_pose_log[index_1][i]);

    return pose;
}

Eigen::Matrix<double,4,4> PoseServer::getTransfAtTime(double t)
{
    std::vector<double> pose = getPoseAtTime(t);
    double y = pose[0];
    double x = pose[1];
    double z = pose[2];
    double roll = pose[3];
    double pitch = pose[4];
    double yaw = pose[5];

    Eigen::Matrix<double,4,4> T_pose;

    // convention from cetin
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

std::tuple<std::vector<double>, int> 
PoseServer::getNearestPoseInLog(const std::vector<double> pose)
{
    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > wrapped_pose;
    wrapped_pose.push_back(pose);
    std::tie(nn_ids, std::ignore) = nearestNeighbors(m_pose_log, wrapped_pose, 1);
    
    return std::make_tuple(m_pose_log[nn_ids[0][0]], nn_ids[0][0]);
}

double PoseServer::getTimeAtIndex(size_t idx)
{
    return m_t_log[idx];
}

size_t PoseServer::getIndexOfNearestTime(double t)
{
    auto it = std::lower_bound(m_t_log.begin(), m_t_log.end(), t);
    size_t index = std::distance(m_t_log.begin(), it) - 1;

    // t greater than max time
    if (index == m_t_log.size())
    {
	std::cout << std::setprecision(20) << t << " outside range, greater than max time" << std::endl;
	throw std::range_error("query time outside range");
    }

    // t less than min time
    if (index == 0)
    {
	std::cout << t << " outside range, less than min time" << std::endl;
	throw std::range_error("query time outside range");
    }

    return index;
}

size_t PoseServer::getNumLogs()
{
    return m_t_log.size();
}
