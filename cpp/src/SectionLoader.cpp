#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

SectionLoader::SectionLoader(std::string rel_path_section) :
    m_num_logs(0),
    m_bracketing_padding(1000)
{
    m_loadSection(rel_path_section);
}

void SectionLoader::m_loadSection(std::string rel_path_section)
{
    m_rel_path_section = rel_path_section;
    std::ifstream section_file(rel_path_section);
    if (!section_file)
    {
	std::stringstream ss_err_msg;
	ss_err_msg << "failed to open file " << rel_path_section;
	throw std::runtime_error(ss_err_msg.str().c_str());
    }

    std::string current_line;
    int prev_packet_id = -1;

    while(std::getline(section_file, current_line))
    {
	std::istringstream iss(current_line);
	int this_packet_id;
	double packet_timestamp_sec;
	double packet_timestamp_nanosec;
	double packet_timestamp;
	
	// packet id and timestamp
	iss >> this_packet_id;
	iss >> packet_timestamp_sec;
	iss >> packet_timestamp_nanosec;
	packet_timestamp = packet_timestamp_sec + packet_timestamp_nanosec*1e-9;

	if (this_packet_id != prev_packet_id)
	{
	    m_packet_ids.push_back(this_packet_id);
	    m_packet_timestamps.push_back(packet_timestamp);
	}

	prev_packet_id = this_packet_id;

	// pt
	std::vector<double> pt(3,0);
	iss >> pt[0];
	iss >> pt[1];
	iss >> pt[2];
	m_pts.push_back(pt);

	m_pt_packet_id.push_back(this_packet_id);
	m_pt_timestamps.push_back(packet_timestamp);
    }
    section_file.close();
    std::cout << "Loaded section: " << rel_path_section << std::endl;
}

std::vector<std::vector<double> > SectionLoader::getPtsAtTime(double t)
{
    std::vector<std::vector<double> > pts;
    for(size_t i = 0; i < m_pt_timestamps.size(); ++i)
	if (m_pt_timestamps[i] == t)
	    pts.push_back(m_pts[i]);

    return pts;    
}

std::vector<int> SectionLoader::getPtIdsAtTime(double t)
{
    std::vector<int> ids;
    for(size_t i = 0; i < m_pt_timestamps.size(); ++i)
	if (m_pt_timestamps[i] == t)
	    ids.push_back(i);

    return ids;    
}

std::vector<std::vector<double> > SectionLoader::getSectionImuPoses(const PoseServer &imu_pose_server)
{
    std::vector<std::vector<double> > section_imu_poses;
    for(size_t i = 0; i < m_packet_timestamps.size(); ++i)
    {
	double t = m_packet_timestamps[i];
	section_imu_poses.push_back(imu_pose_server.getPoseAtTime(t));
    }

    return section_imu_poses;
}

std::vector<std::vector<double> > SectionLoader::getSectionImuPosns(const PoseServer &imu_pose_server)
{
    std::vector<std::vector<double> > section_imu_posns;
    for(size_t i = 0; i < m_packet_timestamps.size(); ++i)
    {
	double t = m_packet_timestamps[i];
	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
	section_imu_posns.push_back(posnFromImuPose(imu_pose));
    }

    return section_imu_posns;
}

std::tuple<int, int> SectionLoader::getLogIdsBracketingImuPosns(const std::vector<std::vector<double> > &imu_posns, const PoseServer &imu_pose_server)
{
    // section imu posns
    std::vector<std::vector<double> > section_imu_posns = getSectionImuPosns(imu_pose_server);

    // nearest section posns to slice posns
    std::vector<std::vector<int> > nn_ids;
    std::tie(nn_ids, std::ignore) = nearestNeighbors(section_imu_posns, imu_posns, 1);

    // slice section log ids
    int start_section_log_id = nn_ids[0][0];
    int end_section_log_id = nn_ids.back()[0];

    // add some padding to log ids
    start_section_log_id = std::max(0, start_section_log_id - m_bracketing_padding);
    end_section_log_id = std::min(end_section_log_id + m_bracketing_padding, (int)m_packet_timestamps.size());

    return std::make_tuple(start_section_log_id, end_section_log_id);
}

