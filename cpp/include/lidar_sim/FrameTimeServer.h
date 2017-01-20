#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class FrameTimeServer {
    public:
	FrameTimeServer(std::string rel_path_frame_time_log);
	size_t getDataIdForFrameId(int frame_id);
	double getTimeAtFrameId(int frame_id);
	void printTimeAtFrameId(int frame_id);

    private:
	void m_loadFrameTimeLog(std::string rel_path_pose_log);
	int getFrameIdFromFilename(std::string filename);
	
	std::vector<double> m_frame_id_log;
	std::vector<double> m_t_log;
	std::string m_path_frame_time_log;
	int m_num_logs;
    };
}

