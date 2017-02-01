#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    typedef std::vector<std::vector<double> > Pts;

    std::vector<double> calcPtsMean(std::vector<std::vector<double> > pts);
    Pts calcCenteredPts(Pts pts);
    Eigen::MatrixXd calcPtsCovMat(std::vector<std::vector<double> > pts);
    Eigen::MatrixXd calcOuterProd(std::vector<double> pt);
    double deg2rad(double angle_deg);
    bool anyNonzeros(std::vector<int> vec);
    std::vector<double> sampleFromMvn(std::vector<double> mu, Eigen::MatrixXd cov_mat);

    /* std::vector<std::vector<double> > logicalSubsetOfArray(std::vector<std::vector<double> > array, std::vector<int> logical_flag) */

    template <typename T>
	std::vector<T> logicalSubset2DArray(std::vector<T> array, std::vector<int> logical_flag)
    {
	std::vector<T> array_subset;
	for(size_t i = 0; i < array.size(); ++i)
	    if (logical_flag[i])
		array_subset.push_back(array[i]);

	return array_subset;
    }

    template <typename T>
	std::vector<int> findNonzeroIds(std::vector<T> vec)
    {
	std::vector<int> nonzero_ids;
	for(size_t i = 0; i < vec.size(); ++i)
	    if (vec[i])
		nonzero_ids.push_back(i);

	return nonzero_ids;
    }

    std::tuple<std::vector<int>, std::vector<double> >
	sortIntersectionFlag(std::vector<int> intersection_flag, std::vector<double> dist_along_ray);

    // nearest neighbor ids for pts2 in pts1
    std::vector<int> nearestNeighbors(std::vector<std::vector<double> > pts1, std::vector<std::vector<double> > pts2);
    std::vector<std::vector<double> > pdist2(std::vector<std::vector<double> > pts1, std::vector<std::vector<double> > pts2);
    double euclideanDist(std::vector<double> pt1, std::vector<double> pt2);
    double calcVariance(std::vector<double> vec);

    std::tuple<int, bool>
	sampleHitId(std::vector<double> hit_prob_vec, std::vector<int> sorted_intersecting_ids);
}

