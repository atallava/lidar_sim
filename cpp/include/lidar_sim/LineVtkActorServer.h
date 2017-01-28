#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyData.h>

#include <Eigen/Dense>

namespace lidar_sim {
    class LineVtkActorServer {
    public:
	LineVtkActorServer();
	vtkSmartPointer<vtkActor> genLineActorPts(std::vector<double> pt1, std::vector<double> pt2);
	vtkSmartPointer<vtkActor> genLineActorDirn(std::vector<double> origin, std::vector<double> dirn);
	vtkSmartPointer<vtkActor> genLineActorPts(double* p0, double* p1);

    private:
	double m_line_length;
	double m_line_width;
	std::vector<double> m_color;
    };
}

