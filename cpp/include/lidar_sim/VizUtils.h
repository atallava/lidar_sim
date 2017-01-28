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
    void dispVec(std::vector<double> vec);
    void dispMat(std::vector<std::vector<double> > mat);
    void dispMat(std::vector<std::vector<int> > mat);
}
