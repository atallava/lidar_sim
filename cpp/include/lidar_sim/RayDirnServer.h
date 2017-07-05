#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class RayDirnServer {
    public:
	RayDirnServer();
	std::tuple<std::vector<double>, std::vector<double> >
	    getRayYawPitchVec(const std::vector<double> &origin, 
			      const std::vector<std::vector<double> > &pts);
	std::vector<double> getPatternYawVec(const std::vector<double> &ray_yaw_vec);
	std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double> >, std::vector<int> >
	    fitDetailToPts(const std::vector<double> &origin, 
					  const std::vector<std::vector<double> > &pts);

	std::vector<double> m_pattern_pitch_vec;
	int m_n_pattern_yaws;
	int m_n_pattern_pitches;
    };
}
