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
	vtkSmartPointer<vtkActor> genEllipsoidActor(std::vector<double> mu, 
						    Eigen::MatrixXd cov_mat, double hit_prob = 1);
	double mapHitProbToOpacity(double hit_prob);
	
    private:
	std::vector<double> m_color;
	double m_max_opacity;
	double m_level;
    };
}

