#pragma once
#include <vector>
#include <string>
#include <fstream>

#include <Eigen/Dense>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/TriangleModels.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    struct EllipsoidModel {
	std::vector<double> mu;
	Eigen::MatrixXd cov_mat;
	double hit_prob;
    };

    typedef std::vector<EllipsoidModel> EllipsoidModels;

    struct EllipsoidModel;
    
    EllipsoidModel createEllipsoidModel(Pts pts);
    void writeEllipsoidModelsToFile(EllipsoidModels ellipsoid_models, std::string rel_path_output);
    EllipsoidModels loadEllipsoidModelsFromFile(std::string rel_path_input);
    void dispEllipsoidModel(EllipsoidModel model);

    void writeTriangleModelsToFile(TriangleModels triangle_models, std::string rel_path_output);
    TriangleModels loadTriangleModelsFromFile(std::string rel_path_input);
    TriangleModels loadMeshModelFromPly(std::string rel_path_input);

    TriangleModels stitchTriangleModels(std::vector<TriangleModels> &triangle_models_vec);
    EllipsoidModels stitchEllipsoidModels(std::vector<EllipsoidModels> &ellipsoid_models_vec);

    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<int> > >
	buildBlocks(const std::vector<std::vector<double> > &imu_posn_nodes,
		    const std::vector<std::vector<double> > &pts, int pts_per_block);

    std::vector<int> getIntersectedFlag(const std::vector<std::vector<int> > &intersection_flag);
    std::tuple<std::vector<int>, std::vector<double> >
	sortIntersectionFlag(const std::vector<int> &intersection_flag, const std::vector<double> &dist_along_ray);
    std::tuple<int, bool>
	sampleHitId(const std::vector<double> &hit_prob_vec, const std::vector<int> &sorted_intersecting_ids, 
		    const bool deterministic_sampling = false);

    std::tuple<std::vector<double>, double> calcRayDirn(const std::vector<double> &ray_origin, const std::vector<double> &end_pt);
    std::vector<std::vector<double> >
	calcRayDirns(const std::vector<double> start_pt, const std::vector<std::vector<double> > end_pts);
    std::vector<std::vector<double> > calcRayDirnsFromSph(const std::vector<double> &ray_pitch_vec, const std::vector<double> &ray_yaw_vec);

    void applyMaxRangeFilter(const std::vector<double> ray_origin,
			     std::vector<std::vector<double> > &sim_pts, std::vector<int> &hit_flag,
			     const double max_range);
    std::vector<std::vector<double> > getNodesAlongRay(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn, 
						       LaserCalibParams laser_calib_params, double node_resn);
    std::vector<std::vector<double> > getNodesAlongRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns, 
							LaserCalibParams laser_calib_params, double node_resn);
}

