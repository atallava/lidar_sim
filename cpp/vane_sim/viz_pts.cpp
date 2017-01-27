#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCamera.h>

#include <lidar_sim/PointsVtkActorServer.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;
 
int main(int, char *[])
{
    // std::string rel_path_input = "../data/taylorJune2014/sections/world_frame/section_pts_01_world_frame_subsampled.xyz";
    std::string rel_path_input = "../data/taylorJune2014/vane/rim_stretch_veg_train.asc";

    // actor
    PointsVtkActorServer points_actor_server;
    vtkSmartPointer<vtkActor> actor = points_actor_server.genPointsActor(rel_path_input);

    // renderer
    vtkSmartPointer<vtkRenderer> renderer =
	vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(.0, .0, .0);
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Elevation(-90);
 
    // render window
    vtkSmartPointer<vtkRenderWindow> renderWindow =
	vtkSmartPointer<vtkRenderWindow>::New();
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
 
    return EXIT_SUCCESS;
}
