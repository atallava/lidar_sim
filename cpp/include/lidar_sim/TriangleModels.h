#pragma once

namespace lidar_sim {
    class TriangleModels {
    public:
	std::vector<std::vector<double> > m_fit_pts;
	std::vector<std::vector<int> > m_triangles;
	std::vector<double> m_hit_prob_vec;
    };
}
