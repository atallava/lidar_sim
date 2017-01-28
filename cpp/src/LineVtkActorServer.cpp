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
#include <vtkLineSource.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/LineVtkActorServer.h>

using namespace lidar_sim;

LineVtkActorServer::LineVtkActorServer() :
    m_line_length(40),
    m_line_width(2)
{
    std::vector<double> ellipse_rgb = {0, 1, 0};
    m_color = ellipse_rgb;
}

vtkSmartPointer<vtkActor> LineVtkActorServer::genLineActorPts(std::vector<double> pt1, std::vector<double> pt2)
{
    double p0[3] = {pt1[0], pt1[1], pt1[2]};
    double p1[3] = {pt2[0], pt2[1], pt2[2]};
    return genLineActorPts(p0, p1);
}

vtkSmartPointer<vtkActor> LineVtkActorServer::genLineActorDirn(std::vector<double> origin, std::vector<double> dirn)
{
    double p0[3] = {origin[0], origin[1], origin[2]};
    double p1[3];
    for(size_t i = 0; i < 3; ++i)
	p1[i] = origin[i] + m_line_length*dirn[i];

    return genLineActorPts(p0, p1);
}

vtkSmartPointer<vtkActor> LineVtkActorServer::genLineActorPts(double* p0, double* p1)
{
    vtkSmartPointer<vtkLineSource> lineSource = 
	vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(p0);
    lineSource->SetPoint2(p1);
    lineSource->Update();
 
    vtkSmartPointer<vtkPolyDataMapper> mapper = 
	vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(lineSource->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = 
	vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(4);
    actor->GetProperty()->SetColor(m_color[0], m_color[1], m_color[2]);

    return actor;
}
