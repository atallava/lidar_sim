#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class RayDirnServer {
    public:
	RayDirnServer();
	std::vector<double> getRayYawPitchVec(const std::vector<double> &origin, 
					      const std::vector<std::vector<double> > &pts);

	std::vector<double> m_pitch_vec;
    };
}
