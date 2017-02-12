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

    double euclideanDist(const std::vector<double> &pt1, const std::vector<double> &pt2);
    double calcVariance(const std::vector<double> &vec);

    std::tuple<int, bool>
	sampleHitId(const std::vector<double> &hit_prob_vec, const std::vector<int> &sorted_intersecting_ids);
    std::vector<int> getIntersectedFlag(const std::vector<std::vector<int> > &intersection_flag);
    std::vector<int> genLogicalVecFromIds(std::vector<int> ids, int vec_size);

    // intervals are [a,b]
    // interval1 ids which overlap with any of interval2
    template <typename T>
	std::vector<int> getOverlappingIntervals(std::vector<std::vector<T> > intervals_1, std::vector<std::vector<T> > intervals_2)
    {
	std::vector<int> ids;
	for(size_t i = 0; i < intervals_1.size(); ++i)
	    for(size_t j = 0; j < intervals_2.size(); ++j)
	    {
		bool condn_1 = ((intervals_2[j][0] <= intervals_1[i][0]) &&
				(intervals_1[i][0] <= intervals_2[j][1]));
		bool condn_2 = ((intervals_2[j][0] <= intervals_1[i][1]) &&
				(intervals_1[i][1] <= intervals_2[j][1]));

		if (condn_1 || condn_2)
		    ids.push_back(i);
	    }

	return ids;
    }

    template<typename T>
	std::vector<T> wrapDataInVec(T data)
    {
	std::vector<T> vec;
	vec.push_back(data);
	
	return vec;
    }

    template<typename T>
	std::tuple<double, double> calcVecMeanVar(std::vector<T> data)
    {
	double mu = std::accumulate(data.begin(), data.end(), 0.0)/(double)data.size();
	double var = 0;
	for(size_t i = 0; i < data.size(); ++i)
	    var += std::pow(data[i]-mu, 2.0);
	var = var/(double)data.size();

	return std::make_tuple(mu, var);
    }

    double vectorNorm(const std::vector<double> &a);
    double dotProduct(const std::vector<double> &a, const std::vector<double> &b);
    std::vector<double> vectorDiff(const std::vector<double> &a, const std::vector<double> &b);
    std::vector<double> getPerpUnitVec2(const std::vector<double> &a);
    std::vector<double> normalizeVec(const std::vector<double> &a);
}

