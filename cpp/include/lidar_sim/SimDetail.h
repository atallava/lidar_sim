#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class SimDetail {
    public:
	SimDetail();
	SimDetail(const std::string rel_path_file);
	void load(const std::string rel_path_sim_detail);

	typedef std::vector<std::vector<double> > Pts;

	std::vector<double> getVecFromLine(const std::string line);
	std::vector<std::vector<double> > getPtsFromLine(const std::string line);
	std::vector<int> getHitFlagFromLine(const std::string line);

	std::vector<std::vector<double> > m_ray_origins;
	std::vector<Pts> m_real_pts;
	std::vector<Pts> m_sim_pts;
	std::vector<std::vector<int> > m_hit_flags;
    };
}
