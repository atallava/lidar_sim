#pragma once
#include <vector>
#include <string>

#include <lidar_sim/SimDetail.h>

namespace lidar_sim {
    class PtsError {
    public:
	PtsError();
	// find pts in pts1 nearest to pts in pts2, and average the distances
	std::tuple<double, double>
	    calcAsymmetricError(const std::vector<std::vector<double> > &pts1, 
				const std::vector<std::vector<double> > &pts2);
	// average of asymmetric error, calculated both ways
	// todo: variance for this error?
	double calcSymmetricError(const std::vector<std::vector<double> > &pts1, 
				  const std::vector<std::vector<double> > &pts2);
	void calcRangeError(const SimDetail &sim_detail);

    private:
    };
}

