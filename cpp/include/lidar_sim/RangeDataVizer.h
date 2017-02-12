#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>

#include <lidar_sim/EllipsoidVtkActorServer.h>
#include <lidar_sim/PointsVtkActorServer.h>
#include <lidar_sim/LineVtkActorServer.h>
#include <lidar_sim/TrianglesVtkActorServer.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/TriangleModeler.h>
#include <lidar_sim/SectionModelSim.h>

namespace lidar_sim {
    class RangeDataVizer {
    public:
	RangeDataVizer();
	void vizEllipsoidModels(const EllipsoidModels &ellipsoid_models);
	void vizEllipsoidModels(const EllipsoidModels &ellipsoid_models, const Pts &pts);
	void vizPts(const std::vector<std::vector<double> > &pts);
	void takeItAway(const std::vector<vtkSmartPointer<vtkActor> > &actors);
	void takeItAway(const vtkSmartPointer<vtkActor> &actor);
	void vizComparePts(const std::vector<std::vector<double> > &pts1,
			   const std::vector<std::vector<double> > &pts2);
	void vizTriangles(const Delaunay_cgal &triangulation,
			  const std::vector<std::vector<double> > &pts);
	void vizTriangles(const std::vector<std::vector<int> > &triangle_vertex_ids,
			  const std::vector<std::vector<double> > &pts);
	void vizSegmentation(const std::vector<std::vector<double> >& pts, const std::vector<int> &segmentation);
	void vizSectionModels(const SectionModelSim &sim);
	std::vector<vtkSmartPointer<vtkActor> >
	    genSectionModelsActors(const SectionModelSim &sim);
	std::vector<vtkSmartPointer<vtkActor> >
	    genTriangleModelsActors(const std::vector<TriangleModelSim> &sims);
	std::vector<vtkSmartPointer<vtkActor> >
	    genEllipsoidModelsActors(const std::vector<EllipsoidModelSim> &sims);
	vtkSmartPointer<vtkActor> genPointsActor(const std::vector<std::vector<double> > &points);

	// pts are required in cyclic order
	vtkSmartPointer<vtkActor> genPolyActor(const std::vector<std::vector<double> > &pts);


	PointsVtkActorServer m_points_actor_server;
	LineVtkActorServer m_line_actor_server;
	EllipsoidVtkActorServer m_ellipsoid_actor_server;
	TrianglesVtkActorServer m_triangles_actor_server;
	std::vector<double> m_brown_color;
	int m_ellipsoid_skip;

    private:
	
    };
}

