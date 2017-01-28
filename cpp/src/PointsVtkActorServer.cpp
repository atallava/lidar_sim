#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <vtkSmartPointer.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkProperty.h>
#include <vtkPropAssembly.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVertexGlyphFilter.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/PointsVtkActorServer.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

PointsVtkActorServer::PointsVtkActorServer() :
    m_point_size(4)
{
}

vtkSmartPointer<vtkActor> PointsVtkActorServer::genPointsActor(std::string rel_path_input)
{
    return genActorFromPolyData(
	genPointsPolyData(rel_path_input));
}

vtkSmartPointer<vtkActor> PointsVtkActorServer::genPointsActor(std::vector<std::vector<double> > points)
{
    return genActorFromPolyData(
	genPointsPolyData(points));
}

vtkSmartPointer<vtkPolyData> PointsVtkActorServer::genPointsPolyData(std::vector<std::vector<double> > points)
{
    vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();
    for(size_t i = 0; i < points.size(); ++i)
	points_vtk->InsertNextPoint(points[i][0], points[i][1], points[i][2]);

    vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPoints(points_vtk);
    return poly_data;
}

vtkSmartPointer<vtkPolyData> PointsVtkActorServer::genPointsPolyData(std::string rel_path_input)
{
    vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();

    // open input file
    std::ifstream input_file(rel_path_input);
    std::cout << "Reading points from: " << rel_path_input << std::endl;

    std::string current_line;
    while (std::getline(input_file, current_line))
    {
	std::istringstream iss(current_line);
	double x, y, z;
	iss >> x;
	iss >> y;
	iss >> z;

	points_vtk->InsertNextPoint(x, y, z);
    }

    vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPoints(points_vtk);
    return poly_data;
}

vtkSmartPointer<vtkActor> PointsVtkActorServer::genActorFromPolyData(vtkSmartPointer<vtkPolyData> poly_data)
{
    // glpyh filter
    vtkSmartPointer<vtkVertexGlyphFilter> vertex_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New(); 
    vertex_glyph_filter->SetInputConnection(poly_data->GetProducerPort()); 
    vertex_glyph_filter->Update();
    
    // mapper
    vtkSmartPointer<vtkPolyDataMapper> mapper =
	vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(vertex_glyph_filter->GetOutputPort());
 
    // actor
    vtkSmartPointer<vtkActor> actor =
	vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(m_point_size);

    return actor;
}

