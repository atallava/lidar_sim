#include <iostream>
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
// #include <vector>

#include "eigenmvn.h"

#include <flann/flann.hpp>

#include "interpolation.h"

#include "gp.h"
#include "gp_utils.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Object.h>
#include <boost/variant.hpp>

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

 
bool Test::testLibgp()
{
    int n=4000, m=1000;
    double tss = 0, error, f, y;
    // initialize Gaussian process for 2-D input using the squared exponential 
    // covariance function with additive white noise.
    libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
    // initialize hyper parameter vector
    Eigen::VectorXd params(gp.covf().get_param_dim());
    params << 0.0, 0.0, -2.0;
    // set parameters of covariance function
    gp.covf().set_loghyper(params);
    // add training patterns
    for(int i = 0; i < n; ++i) {
	double x[] = {drand48()*4-2, drand48()*4-2};
	y = libgp::Utils::hill(x[0], x[1]) + libgp::Utils::randn() * 0.1;
	gp.add_pattern(x, y);
    }
    // total squared error
    for(int i = 0; i < m; ++i) {
	double x[] = {drand48()*4-2, drand48()*4-2};
	f = gp.f(x);
	y = libgp::Utils::hill(x[0], x[1]);
	error = f - y;
	tss += error*error;
    }
    std::cout << "mse = " << tss/m << std::endl;
    return EXIT_SUCCESS;
}

bool Test::testAlglibRbf()
{
    //
    // This example shows how to build models with RBF-ML algorithm. Below
    // we assume that you already know basic concepts shown in the example
    // on RBF-QNN algorithm.
    //
    // RBF-ML is a multilayer RBF algorithm, which fits a sequence of models
    // with decreasing radii. Each model is fitted with fixed number of
    // iterations of linear solver. First layers give only inexact approximation
    // of the target function, because RBF problems with large radii are
    // ill-conditioned. However, as we add more and more layers with smaller
    // and smaller radii, we get better conditioned systems - and more precise models.
    //
    alglib::rbfmodel model;
    alglib::rbfreport rep;
    double v;

    //
    // We have 2-dimensional space and very simple interpolation problem - all
    // points are distinct and located at straight line. We want to solve it
    // with RBF-ML algorithm. This problem is very simple, and RBF-QNN will
    // solve it too, but we want to evaluate RBF-ML and to start from the simple
    // problem.
    //     X        Y
    //     -2       1
    //     -1       0
    //      0       1
    //     +1      -1
    //     +2       1
    //
    alglib::rbfcreate(2, 1, model);
    alglib::real_2d_array xy0 = "[[-2,0,1],[-1,0,0],[0,0,1],[+1,0,-1],[+2,0,1]]";
    alglib::rbfsetpoints(model, xy0);

    // First, we try to use R=5.0 with single layer (NLayers=1) and moderate amount
    // of regularization.... but results are disappointing: Model(x=0,y=0)=-0.02,
    // and we need 1.0 at (x,y)=(0,0). Why?
    //
    // Because first layer gives very smooth and imprecise approximation of the
    // function. Average distance between points is 1.0, and R=5.0 is too large
    // to give us flexible model. It can give smoothness, but can't give precision.
    // So we need more layers with smaller radii.
    alglib::rbfsetalgomultilayer(model, 5.0, 1, 1.0e-3);
    alglib::rbfbuildmodel(model, rep);
    printf("%d\n", int(rep.terminationtype)); // EXPECTED: 1
    v = alglib::rbfcalc2(model, 0.0, 0.0);
    printf("%.2f\n", double(v)); // EXPECTED: -0.021690

    // Now we know that single layer is not enough. We still want to start with
    // R=5.0 because it has good smoothness properties, but we will add more layers,
    // each with R[i+1]=R[i]/2. We think that 4 layers is enough, because last layer
    // will have R = 5.0/2^3 = 5/8 ~ 0.63, which is smaller than the average distance
    // between points. And it works!
    alglib::rbfsetalgomultilayer(model, 5.0, 4, 1.0e-3);
    alglib::rbfbuildmodel(model, rep);
    printf("%d\n", int(rep.terminationtype)); // EXPECTED: 1
    v = alglib::rbfcalc2(model, 0.0, 0.0);
    printf("%.2f\n", double(v)); // EXPECTED: 1.000000

    // BTW, if you look at v, you will see that it is equal to 0.9999999997, not to 1.
    // This small error can be fixed by adding one more layer.

    return true;
}

bool Test::testCgal()
{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
    typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
    typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
    typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                    Delaunay;
    typedef Kernel::Point_2                                                Point;

    std::vector< std::pair<Point,unsigned> > points;
    points.push_back( std::make_pair( Point(1,1), 0 ) );
    points.push_back( std::make_pair( Point(1,2), 1 ) );
    points.push_back( std::make_pair( Point(1,3), 2 ) );
    points.push_back( std::make_pair( Point(2,1), 3 ) );
    points.push_back( std::make_pair( Point(2,2), 4 ) );
    points.push_back( std::make_pair( Point(2,3), 5 ) );

    Delaunay triangulation;
    triangulation.insert(points.begin(),points.end());

    for(Delaunay::Finite_faces_iterator fit = triangulation.finite_faces_begin();
	fit != triangulation.finite_faces_end(); ++fit) {

	Delaunay::Face_handle face = fit;
	std::cout << "Triangle:\t" << triangulation.triangle(face) << std::endl;
	std::cout << "Vertex 0:\t" << triangulation.triangle(face)[0] << std::endl;
	std::cout << "Vertex 0:\t" << face->vertex(0)->info() << std::endl;
    }

    return true;
}

bool Test::testCgalIntersection()
{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
    typedef CGAL::Ray_3<Kernel> Ray_3;
    typedef CGAL::Point_3<Kernel> Point_3;
    typedef CGAL::Direction_3<Kernel> Direction_3;
    typedef Kernel::Triangle_3 Triangle_3;
    
    // ray along x-axis
    Point_3 ray_origin(0, 0, 0);
    Direction_3 ray_dirn(1, 0, 0);
    Ray_3 ray(ray_origin, ray_dirn);

    // triangle 1 that ray should hit
    Point_3 v11(1, 0, 1);
    Point_3 v12(1, 1, -1);
    Point_3 v13(1, -1, -1);
    Triangle_3 t1(v11, v12, v13);

    CGAL::Object obj = intersection(ray, t1);

    Point_3 pt_intersection;
    if (assign(pt_intersection, obj))
    {
	std::cout << "hit! point:" << std::endl;
	std::cout << pt_intersection << std::endl;
    }

    // triangle 2 that ray should not hit
    Point_3 v21(1, 0, 10);
    Point_3 v22(1, 1, -9);
    Point_3 v23(1, -1, -9);
    Triangle_3 t2(v21, v22, v23);

    obj = intersection(ray, t2);

    if (assign(pt_intersection, obj))
    {
	std::cout << "hit! point:" << std::endl;
	std::cout << pt_intersection << std::endl;
    }

    return true;
}
