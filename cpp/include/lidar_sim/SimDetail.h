#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class SimDetail {
    public:
	SimDetail();
	SimDetail(const std::string rel_path_file);
	void load(const std::string rel_path_sim_detail);
	void save(const std::string rel_path_sim_detail);
	void writeSimPackets(const std::string rel_path_sim_packets);
	void setVerbosity(int verbose);

	typedef std::vector<std::vector<double> > Pts;

	std::vector<double> getVecFromLine(const std::string line);
	std::vector<std::vector<double> > getPtsFromLine(const std::string line);
	std::vector<int> getHitFlagFromLine(const std::string line);

	std::vector<int> m_packet_ids;
	std::vector<double> m_packet_timestamps;
	std::vector<std::vector<double> > m_ray_origins;
	std::vector<std::vector<double> > m_ray_pitches;
	std::vector<std::vector<double> > m_ray_yaws;
	std::vector<Pts> m_real_pts_all;
	std::vector<std::vector<int> > m_real_hit_flags;
	std::vector<Pts> m_sim_pts_all;
	std::vector<std::vector<int> > m_sim_hit_flags;

    private:
	int m_verbose;
    };
}
