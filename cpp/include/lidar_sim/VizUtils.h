#pragma once
#include <string>

#include <Eigen/Dense>

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>

namespace lidar_sim {
    vtkSmartPointer<vtkPoints> getVtkPointsFromXYZ(std::string rel_path_input);
    vtkSmartPointer<vtkMatrix4x4> getVtkTransform(Eigen::MatrixXd T);
    vtkSmartPointer<vtkMatrix4x4> getVtkTransform(Eigen::MatrixXd R, std::vector<double> t);

    template<typename T>
    void dispVec(std::vector<T> vec)
    {
	for(size_t i = 0; i < vec.size(); ++i)
	    std::cout << vec[i] << " ";
	std::cout << std::endl;
    }

    template<typename T>
    void dispMat(std::vector<std::vector<T> > mat)
    {
	for(size_t i = 0; i < mat.size(); ++i)
	{
	    for(size_t j = 0; j < mat[i].size(); ++j)
		std::cout << mat[i][j] << " ";
	    std::cout << std::endl;
	}
    }

    void dispHorizontalLine(int length = 50);
}
