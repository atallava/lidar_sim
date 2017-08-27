#pragma once

namespace lidar_sim {
    class OptimProgress {
    public:
	OptimProgress();
	void log(const std::vector<double>& x, const double obj, const double elapsed_time);
	void save(const std::string rel_path_output);
	
	std::vector<std::vector<double> > m_x;
	std::vector<double> m_obj;
	std::vector<double> m_t;
	bool m_verbose;

    private:
    };
}
	
