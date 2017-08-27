#pragma once

namespace lidar_sim {
    class OptimProgress {
    public:
	OptimProgress();
	void log(const std::vector<double>& x, const double J, const double elapsed_time);
	void save(const std::string rel_path_output);
	
	std::vector<std::vector<double> > m_x;
	std::vector<double> m_J;
	std::vector<double> m_t;
	bool m_verbose;

    private:
    };
}
	
