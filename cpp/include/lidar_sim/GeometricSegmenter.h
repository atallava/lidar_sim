#pragma once
#include <vector>
#include <string>
#include <cmath>

namespace lidar_sim {
    class GeometricSegmenter {
    public:
	GeometricSegmenter();
	void setDebugFlag(int flag);
	std::vector<int> segmentPts(const std::vector<std::vector<double> > &pts);
	std::vector<double> calcFeaturesForPts(const std::vector<std::vector<double> > &pts);
	double calcSphericalVariation(const std::vector<std::vector<double> > &pts);
	std::vector<double> smoothFeatures(const std::vector<double> &features, const std::vector<std::vector<double> > &pts);
	double calcSmoothedFeature(const std::vector<double> &pt, const double feature, const std::vector<std::vector<double> > &nbrs, const std::vector<double> &nbrs_features);
	double calcSmoothingWeight(const std::vector<double> &pt, const double feature, 
				   const std::vector<double> &nbr, const double nbr_feature);

    private:
	double m_max_dist2_to_nbr;
	double m_min_nbrs;
	double m_default_spherical_variation;
	double m_spherical_variation_threshold;
	int m_debug_flag;
	int m_smooth_features;
	double m_smoothing_distance_weight;
	double m_smoothing_score_weight;
	double m_smoothing_max_nbrs;
	double m_smoothing_max_dist;
    };
}
