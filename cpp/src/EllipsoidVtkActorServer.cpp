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
#include <vtkActor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkProperty.h>
#include <vtkPropAssembly.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/EllipsoidVtkActorServer.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

EllipsoidVtkActorServer::EllipsoidVtkActorServer() :
    m_opacity(0.3),
    m_level(9)
{
    // ellipsoids are green, for vegetation
    std::vector<double> ellipse_rgb = {0, 1, 0};
    m_color = ellipse_rgb;
}

vtkSmartPointer<vtkActor> EllipsoidVtkActorServer::genEllipsoidActor(std::vector<double> mu, 
						       Eigen::MatrixXd cov_mat)
{
    // parametric ellipsoid
    vtkSmartPointer<vtkParametricFunction> 
	parametricObject = vtkSmartPointer<vtkParametricEllipsoid>::New();
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetXRadius(1.0);
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetYRadius(1.0);
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetZRadius(1.0);
    vtkSmartPointer<vtkParametricFunctionSource> 
    	parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->SetUResolution(51);
    parametricFunctionSource->SetVResolution(51);
    parametricFunctionSource->SetWResolution(51);
    parametricFunctionSource->Update();

    // transformation matrix
    Eigen::LLT<Eigen::MatrixXd> llt_cov_mat(cov_mat.inverse()/m_level);
    Eigen::MatrixXd L = llt_cov_mat.matrixL();
    Eigen::MatrixXd R = L.transpose();
    Eigen::MatrixXd R_inv = R.inverse();
    vtkSmartPointer<vtkMatrix4x4> T_model_world = 
	getVtkTransform(R_inv, mu);

    // transform filter
    vtkSmartPointer<vtkTransform> tf =
	vtkSmartPointer<vtkTransform>::New();
    tf->SetMatrix(T_model_world);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
	vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(parametricFunctionSource->GetOutputPort());
    transformFilter->SetTransform(tf);
    transformFilter->Update();
    
    // mapper
    vtkSmartPointer<vtkPolyDataMapper>
    	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(
    	transformFilter->GetOutputPort());
 
    // actor
    vtkSmartPointer<vtkActor>
    	actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(m_color[0], m_color[1], m_color[2]);
    actor->GetProperty()->SetOpacity(m_opacity);

    return actor;
}


