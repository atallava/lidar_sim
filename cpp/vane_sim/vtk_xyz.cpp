#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>

#include <lidar_sim/Utils.h>

using namespace lidar_sim;
 
int main(int, char *[])
{
    // std::string rel_path_xyz = "../data/taylorJune2014/sections/world_frame/section_pts_01_world_frame_subsampled.xyz";
    std::string rel_path_xyz = "../data/taylorJune2014/vane/rim_stretch_veg_train.asc";

    // Create a point cloud
    vtkPolyData* polyData = vtkPolyData::New();
    polyData->SetPoints(getVtkPointsFromXYZ(rel_path_xyz));

    vtkVertexGlyphFilter* glyphFilter = vtkVertexGlyphFilter::New(); 
    glyphFilter->SetInputConnection(polyData->GetProducerPort()); 

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
	vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());
 
    vtkSmartPointer<vtkActor> actor =
	vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
 
    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer =
	vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
	vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
 
    // Add the actor to the scene
    renderer->AddActor(actor);
    // renderer->SetBackground(.3, .6, .3); // Background color green
    renderer->SetBackground(.0, .0, .0); // Background color green
 
    // Render and interact
    renderWindow->Render();
    renderWindowInteractor->Start();
 
    return EXIT_SUCCESS;
}
