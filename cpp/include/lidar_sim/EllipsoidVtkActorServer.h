#pragma once
#include <vector>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <Eigen/Dense>

namespace lidar_sim {
    class EllipsoidVtkActorServer {
    public:
	EllipsoidVtkActorServer();
	vtkSmartPointer<vtkActor> genEllipsoidVtkActor(std::vector<double> mu, 
						       Eigen::MatrixXd cov_mat);

    private:
	std::vector<double> m_color;
	double m_opacity;
	double m_level;
    };
}

