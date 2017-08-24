#include <iostream>
#include <sstream>

#include <lidar_sim/VizUtils.h>

#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>

namespace lidar_sim {
    vtkSmartPointer<vtkPoints> getVtkPointsFromXYZ(std::string rel_path_input)
    {
	// open input file
	std::ifstream input_file(rel_path_input);
	std::cout << "Reading from: " << rel_path_input << std::endl;

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	std::string current_line;
	while (std::getline(input_file, current_line))
	{
	    std::istringstream iss(current_line);
	    double x, y, z;
	    iss >> x;
	    iss >> y;
	    iss >> z;

	    points->InsertNextPoint(x, y, z);
  	}
  
	return points;
    }

    vtkSmartPointer<vtkMatrix4x4> getVtkTransform(Eigen::MatrixXd T)
    {
	auto T_vtk = 
	    vtkSmartPointer<vtkMatrix4x4>::New();
	for(size_t i = 0; i < 4; ++i)
	    for(size_t j = 0; j < 4; ++j)
		T_vtk->SetElement(i,j,T(i,j));

	return T_vtk;
    }

    vtkSmartPointer<vtkMatrix4x4> getVtkTransform(Eigen::MatrixXd R, std::vector<double> t)
    {
	auto T_vtk = 
	    vtkSmartPointer<vtkMatrix4x4>::New();

	// rotation
	for(size_t i = 0; i < 3; ++i)
	    for(size_t j = 0; j < 3; ++j)
		T_vtk->SetElement(i,j,R(i,j));

	// translation
	for(size_t i = 0; i < 3; ++i)
	{
	    T_vtk->SetElement(i,3,t[i]);
	    T_vtk->SetElement(3,i,0);
	}

	T_vtk->SetElement(3,3,1);

	return T_vtk;
    }

    void dispHorizontalLine(int length)
    {
    	std::cout << std::endl;
    	for(size_t i = 0; i < (size_t)length; ++i)
    	    std::cout << "-";
    	std::cout << std::endl << std::endl;
    }
}
