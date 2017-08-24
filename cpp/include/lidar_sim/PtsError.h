#pragma once
#include <vector>
#include <string>
#include <tuple>

#include <lidar_sim/SimDetail.h>
#include <lidar_sim/MathUtils.h>

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

	template<typename T>
	    void dispVecMeanVar(std::vector<T> vec)
	{
	    double mean, var;
	    std::tie(mean, var) = calcVecMeanVar(vec);
	    std::cout << "mean: " << mean << ", var: " << var << std::endl;
	}

    private:
    };
}

