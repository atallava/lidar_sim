#pragma once
#include <vector>
#include <string>

#include <lidar_sim/SimDetail.h>

namespace lidar_sim {
    class PtsError {
    public:
	PtsError();
	// find pts in 1 nearest to pts in 2. pts1 is the 'reference'.
	std::tuple<double, double>
	    calcAsymmetricPcdError(const std::vector<std::vector<double> > &pts1, 
				   const std::vector<std::vector<double> > &pts2);
	// average of asymmetric error, calculated both ways
	// todo: variance for this error?
	double calcSymmetricPcdError(const std::vector<std::vector<double> > &pts1, 
				     const std::vector<std::vector<double> > &pts2);
	void dispPcdError(const std::vector<std::vector<double> > &pts1, 
				    const std::vector<std::vector<double> > &pts2);
	void dispRangeError(const SimDetail &sim_detail);

    private:
    };
}

