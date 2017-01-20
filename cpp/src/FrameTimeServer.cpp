#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/FrameTimeServer.h>

using namespace lidar_sim;

FrameTimeServer::FrameTimeServer(std::string rel_path_frame_time_log) :
    m_path_frame_time_log(rel_path_frame_time_log),
    m_num_logs(0)
{
    m_loadFrameTimeLog(m_path_frame_time_log);
}

void FrameTimeServer::m_loadFrameTimeLog(std::string path_frame_time_log)
{
    std::ifstream frame_time_log_file(path_frame_time_log);
    std::string current_line;
    m_num_logs = 0;

    // first line is header
    std::getline(frame_time_log_file, current_line); 

    while(std::getline(frame_time_log_file, current_line))
    {
	std::istringstream iss(current_line);
	std::string token;

	std::getline(iss, token, ',');
	int frame_id = getFrameIdFromFilename(token);
	m_frame_id_log.push_back(frame_id);

	std::getline(iss, token, ',');
	double t = std::stod(token);
	m_t_log.push_back(t);
    }
    frame_time_log_file.close();
    std::cout << "Loaded frame times from: " << path_frame_time_log << std::endl;
}

int FrameTimeServer::getFrameIdFromFilename(std::string filename)
{
    std::string frame_id_str = filename.substr(14,6);
    int frame_id = std::stoi(frame_id_str);
    return frame_id;
}


size_t FrameTimeServer::getDataIdForFrameId(int frame_id)
{
    auto it = std::find(m_frame_id_log.begin(), m_frame_id_log.end(), frame_id);
    size_t data_id = std::distance(m_frame_id_log.begin(), it);
    return data_id;
}

double FrameTimeServer::getTimeAtFrameId(int frame_id)
{
    size_t data_id = getDataIdForFrameId(frame_id);
    return m_t_log[data_id];
}

void FrameTimeServer::printTimeAtFrameId(int frame_id)
{
    std::cout << "id: " << frame_id << std::endl;
    std::cout << std::setprecision(20) << "t: " << getTimeAtFrameId(frame_id) << std::endl;
}
