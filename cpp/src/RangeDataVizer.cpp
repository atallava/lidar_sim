#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkTextMapper.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkParametricEllipsoid.h>
#include <vtkPropAssembly.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

RangeDataVizer::RangeDataVizer() :
    m_points_actor_server(),
    m_line_actor_server(),
    m_ellipsoid_actor_server()
{    
}

void RangeDataVizer::vizEllipsoidModels(EllipsoidModels ellipsoid_models)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    size_t n_ellipsoids = ellipsoid_models.size();

    std::vector<double> mu(3,0);
    Eigen::MatrixXd cov_mat(3,3);
    cov_mat << 
	1,0,0,
	0,1,0,
	0,0,1;
    actors.push_back(
    	m_ellipsoid_actor_server.genEllipsoidActor(mu, cov_mat));
    for(size_t i = 0; i < n_ellipsoids; ++i)
    {
    	actors.push_back(
    	    m_ellipsoid_actor_server.genEllipsoidActor(ellipsoid_models[i].mu, 
    						       ellipsoid_models[i].cov_mat));
    }

    takeItAway(actors);
}

void RangeDataVizer::vizEllipsoidModels(EllipsoidModels ellipsoid_models, Pts pts)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    size_t n_ellipsoids = ellipsoid_models.size();

    // origin dot
    std::vector<double> origin_dot_mu(3,0);
    Eigen::MatrixXd origin_dot_cov_mat(3,3);
    origin_dot_cov_mat << 
	1,0,0,
	0,1,0,
	0,0,1;
    actors.push_back(
    	m_ellipsoid_actor_server.genEllipsoidActor(origin_dot_mu, origin_dot_cov_mat));

    // ellipsoids
    for(size_t i = 0; i < n_ellipsoids; ++i)
    {
    	actors.push_back(
    	    m_ellipsoid_actor_server.genEllipsoidActor(ellipsoid_models[i].mu, 
    						       ellipsoid_models[i].cov_mat));
    }

    // pts
    actors.push_back(
	m_points_actor_server.genPointsActor(pts));
	
    takeItAway(actors);
}

void RangeDataVizer::takeItAway(std::vector<vtkSmartPointer<vtkActor> > actors)
{
    // renderer
    vtkSmartPointer<vtkRenderer>
    	renderer = vtkSmartPointer<vtkRenderer>::New();

    // TODO: how to decide renderer size?
    int renderer_size = 500;

    // (xmin, ymin, xmax, ymax)
    // double viewport[4] =
    // 	{static_cast<double>(0),
    // 	 static_cast<double>(0),
    // 	 static_cast<double>(renderer_size),
    // 	 static_cast<double>(renderer_size)};
    // renderer->SetViewport(viewport);

    for(size_t i = 0; i < actors.size(); ++i)
	renderer->AddActor(actors[i]);
    renderer->SetBackground(0, 0, 0);
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Elevation(-90);
    renderer->GetActiveCamera()->Zoom(0.9);
    renderer->ResetCameraClippingRange();

    // render window
    vtkSmartPointer<vtkRenderWindow> renderWindow =
    	vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(
    	renderer_size, renderer_size);
    renderWindow->AddRenderer(renderer);

    // interactor
    vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    	vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    // coordinate axes
    vtkSmartPointer<vtkAxesActor> axes = 
    	vtkSmartPointer<vtkAxesActor>::New();
 
    vtkSmartPointer<vtkOrientationMarkerWidget> widget = 
    	vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
    widget->SetOrientationMarker( axes );
    widget->SetInteractor( interactor );
    double axes_viewport[4] = {0.0, 0.0, 0.4, 0.4};
    widget->SetViewport(axes_viewport[0], axes_viewport[1], axes_viewport[2], axes_viewport[3]);
    widget->SetEnabled( 1 );
    widget->InteractiveOn();

    // interactor style
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
	vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); 
    interactor->SetInteractorStyle( style );
 
    // fire up
    renderWindow->Render();
    interactor->Start();
}
