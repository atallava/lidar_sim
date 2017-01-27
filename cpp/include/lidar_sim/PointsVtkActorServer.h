#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>

namespace lidar_sim {
    class PointsVtkActorServer {
    public:
	PointsVtkActorServer();
	vtkSmartPointer<vtkActor> genPointsActor(std::vector<std::vector<double> > points);
	vtkSmartPointer<vtkActor> genPointsActor(std::string rel_path_input);	
	vtkSmartPointer<vtkPolyData> genPointsPolyData(std::vector<std::vector<double> > points);
	vtkSmartPointer<vtkPolyData> genPointsPolyData(std::string rel_path_input);	
	vtkSmartPointer<vtkActor> genActorFromPolyData(vtkSmartPointer<vtkPolyData> poly_data);

    private:
	double m_point_size;
    };
}

