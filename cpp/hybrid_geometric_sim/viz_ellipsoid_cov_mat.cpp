#include <vector>
 
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

#include <lidar_sim/EllipsoidVtkActorServer.h>

using namespace lidar_sim;

int main(int, char *[])
{
    // specify a transformation matrix
    Eigen::MatrixXd cov_mat(3,3);
    cov_mat << 
	7, 0, 0,
	0, 11, -5,
	0, -5, 9;
    std::vector<double> mu = {0, 0, 0};

    Eigen::LLT<Eigen::MatrixXd> llt_cov_mat(cov_mat);
    Eigen::MatrixXd L = llt_cov_mat.matrixL();
    Eigen::MatrixXd R = L.transpose();

    std::cout << "cov mat: " << std::endl 
	      << cov_mat << std::endl;
    
    std::cout << "L: " << std::endl 
	      << L << std::endl;

    std::cout << "R: " << std::endl 
	      << R << std::endl;

    // ellipsoid actor server
    EllipsoidVtkActorServer ellipsoid_actor_server;

    // get actor
    vtkSmartPointer<vtkActor> actor = ellipsoid_actor_server.genEllipsoidActor(mu, cov_mat);

    // renderer
    vtkSmartPointer<vtkRenderer>
    	renderer = vtkSmartPointer<vtkRenderer>::New();
    int renderer_size = 500;
    // (xmin, ymin, xmax, ymax)
    double viewport[4] =
	{static_cast<double>(0),
	 static_cast<double>(0),
	 static_cast<double>(renderer_size),
	 static_cast<double>(renderer_size)};
    renderer->SetViewport(viewport);
    renderer->AddActor(actor);
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

    return 0;
}
