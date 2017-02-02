#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class GeometricSegmenter {
    public:
	GeometricSegmenter();
	void setDebugFlag(int flag);
	std::vector<int> segmentPts(const std::vector<std::vector<double> > &pts);
	std::vector<double> calcFeaturesForPts(const std::vector<std::vector<double> > &pts);
	double calcSphericalVariation(const std::vector<std::vector<double> > &pts);

    private:
	double m_max_dist2_to_nbr;
	double m_min_nbrs;
	double m_default_spherical_variation;
	double m_spherical_variation_threshold;
	int m_debug_flag;
    };
}
