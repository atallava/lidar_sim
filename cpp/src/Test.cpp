#include <iostream>

#include "eigenmvn.h"

#include <flann/flann.hpp>

#include <lidar_sim/Test.h>
#include <lidar_sim/Visualizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

bool Test::testVizPCD(std::string file_name, int is_rgb = 1)
{
    std::cout << "Visualizing: " << file_name << std::endl;
    Visualizer viz;
    viz.visualize(file_name, is_rgb);
    return true;
}

bool Test::testPoseServer(std::string rel_path_pose_log)
{
    PoseServer pose_server(rel_path_pose_log);
    for(size_t i = 0; i < 10; i++)
    	pose_server.printLogAtIndex(i);    
    return true;
}

bool Test::testGetPoseAtTime(std::string rel_path_pose_log, double t)
{
    PoseServer pose_server(rel_path_pose_log);
    std::vector<double> pose = pose_server.getPoseAtTime(t);
    pose_server.printPose(pose);
    return true;
}

bool Test::testCovMat()
{
    Pts pts { {1,2,3},
	{6,7,6},
	{5,1,5},
	{9,7,8}};
    
    std::vector<double> mu = calcPtsMean(pts);
    std::cout << "mu :" << std::endl;
    for(size_t i = 0; i < mu.size(); ++i)
	std::cout << mu[i] << " ";
    std::cout << std::endl;

    Pts centered_pts = calcCenteredPts(pts);
    std::cout << "centered :" << std::endl;
    for(size_t j = 0; j < centered_pts.size(); ++j)
    {
	for(size_t i = 0; i < 3; ++i)
	    std::cout << centered_pts[j][i] << " ";
	std::cout << std::endl;
    }

    Eigen::MatrixXd cov_mat = calcPtsCovMat(pts);
    std::cout << "cov mat: " << cov_mat << std::endl;

    return true;
}

bool Test::testEigenmvn()
{
    Pts pts { {1,2,3},
	{6,7,6},
	{5,1,5},
	{9,7,8}};
    
    std::vector<double> mu = calcPtsMean(pts);
    Eigen::MatrixXd mean(3,1);
    for(size_t i = 0; i < 3; ++i)
	mean(i) = mu[i];
    Eigen::MatrixXd cov_mat = calcPtsCovMat(pts);
    
    Eigen::EigenMultivariateNormal<double> normX_solver(mean, cov_mat);

    Eigen::MatrixXd samples = normX_solver.samples(20);

    std::cout << "samples: " << std::endl;
    std::cout << samples << std::endl;
    
    return true;
}

bool Test::testFLANN()
{
    std::vector<std::vector<double> > pts1 = 
	{{0, 0},
	 {0, 1},
	 {1, 0}};
      
    flann::Matrix<double> dataset(new double[pts1.size()*pts1[0].size()], pts1.size(), pts1[0].size());
    for(size_t i = 0; i < pts1.size(); ++i)
	for(size_t j = 0; j < pts1[0].size(); ++j)
	    dataset[i][j] = pts1[i][j];

    std::vector<std::vector<double> > pts2 = 
	{{0.6, 0},
	 {0, 0.8}};
    flann::Matrix<double> query(new double[pts2.size()*pts2[0].size()], pts2.size(), pts2[0].size());
    for(size_t i = 0; i < pts2.size(); ++i)
    	for(size_t j = 0; j < pts2[0].size(); ++j)
    	    query[i][j] = pts2[i][j];
    
    int nn = 2;
    
    flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);


    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();                                                                                               

    // do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));

    std::vector<int> nn_ids;
    for(size_t i = 0; i < pts2.size(); ++i)
	nn_ids.push_back(indices[i][0]);

    dispVec(nn_ids);

    return true;
}
