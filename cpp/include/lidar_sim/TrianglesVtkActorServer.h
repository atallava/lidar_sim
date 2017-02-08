#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyData.h>

#include <Eigen/Dense>

#include <lidar_sim/TriangleModeler.h>

namespace lidar_sim {
    class TrianglesVtkActorServer {
    public:
	TrianglesVtkActorServer();
	vtkSmartPointer<vtkActor> genTrianglesActor(const Delaunay_cgal &triangulation, 
						    const std::vector<std::vector<double> > &m_fit_pts);
	vtkSmartPointer<vtkActor> genTrianglesActor(const std::vector<std::vector<int> > &triangle_vertex_ids, 
						    const std::vector<std::vector<double> > &pts);
	std::vector<vtkSmartPointer<vtkActor> > 
	    genTrianglesActors(const std::vector<std::vector<int> > &triangle_vertex_ids, 
			       const std::vector<std::vector<double> > &pts, const std::vector<double> hit_prob_vec);
	std::vector<vtkSmartPointer<vtkActor> > 
	    genTrianglesActors(const std::vector<std::vector<int> > &triangle_vertex_ids, 
			       const std::vector<std::vector<double> > &pts);

	double mapHitProbToOpacity(double hit_prob);

    private:
	double m_max_opacity;
	std::vector<double> m_brown_color;
    };
}

