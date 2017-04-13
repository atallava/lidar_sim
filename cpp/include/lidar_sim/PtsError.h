#pragma once
#include <vector>
#include <string>

namespace lidar_sim {
    class PtsError {
    public:
	PtsError();
	std::tuple<double, double>
	    calcError(const std::vector<std::vector<double> > &pts1, 
		      const std::vector<std::vector<double> > &pts2);

    private:
    };
}

