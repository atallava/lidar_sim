#pragma once

namespace lidar_sim {
    class OptimProgress {
    public:
	OptimProgress();
	void save(const std::string rel_path_output);
	
	std::vector<std::vector<double> > m_x;
	std::vector<double> m_J;
	std::vector<double> m_t;
	bool m_verbose;

    private:
    };
}
	
