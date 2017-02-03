#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    typedef std::vector<std::vector<double> > Pts;

    std::vector<double> calcPtsMean(const std::vector<std::vector<double> > &pts);
    Pts calcCenteredPts(const Pts &pts);
    Eigen::MatrixXd calcPtsCovMat(const std::vector<std::vector<double> > &pts);
    Eigen::MatrixXd calcOuterProd(const std::vector<double> &pt);
    double deg2rad(double angle_deg);
    bool anyNonzeros(const std::vector<int> &vec);
    std::vector<double> sampleFromMvn(const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat);

    // mimicking matlab mat(flag,:)
    template <typename T>
	std::vector<T> logicalSubsetArray(const std::vector<T> &array, const std::vector<int> &logical_flag)
    {
	std::vector<T> array_subset;
	for(size_t i = 0; i < array.size(); ++i)
	    if (logical_flag[i])
		array_subset.push_back(array[i]);

	return array_subset;
    }

    template <typename T>
	std::vector<int> findNonzeroIds(const std::vector<T> &vec)
    {
	std::vector<int> nonzero_ids;
	for(size_t i = 0; i < vec.size(); ++i)
	    if (vec[i])
		nonzero_ids.push_back(i);

	return nonzero_ids;
    }

    std::tuple<std::vector<int>, std::vector<double> >
	sortIntersectionFlag(const std::vector<int> &intersection_flag, const std::vector<double> &dist_along_ray);

    std::vector<int> negateLogicalVec(const std::vector<int> &vec);

    // nearest neighbor ids for pts2 in pts1
    std::tuple<std::vector<int>, std::vector<double> >
	nearestNeighbors(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2);
    std::vector<std::vector<double> > pdist2(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2);
    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
	nearestNeighbors(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2, const int nn);

    double euclideanDist(std::vector<double> pt1, std::vector<double> pt2);
    double calcVariance(const std::vector<double> &vec);

    std::tuple<int, bool>
	sampleHitId(const std::vector<double> &hit_prob_vec, const std::vector<int> &sorted_intersecting_ids);
    std::vector<int> getIntersectedFlag(const std::vector<std::vector<int> > &intersection_flag);
}

