#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyData.h>

#include <Eigen/Dense>

#include <lidar_sim/GroundModeler.h>

namespace lidar_sim {
    class TrianglesVtkActorServer {
    public:
	TrianglesVtkActorServer();
	vtkSmartPointer<vtkActor> genTrianglesActor(const Delaunay_cgal &triangulation, 
						    const std::vector<std::vector<double> > &m_fit_pts);

    private:
    };
}

