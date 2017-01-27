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
 
#include <Eigen/Dense>

int main(int, char *[])
{
    vtkSmartPointer<vtkParametricFunction> 
	parametricObject = vtkSmartPointer<vtkParametricEllipsoid>::New();
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetXRadius(2.0);
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetYRadius(1.0);
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetZRadius(1.0);
 
    // Create one text property for all
    vtkSmartPointer<vtkTextProperty> textProperty =
    	vtkSmartPointer<vtkTextProperty>::New();
    textProperty->SetFontSize(10);
    textProperty->SetJustificationToCentered();
 
    vtkSmartPointer<vtkProperty> backProperty =
    	vtkSmartPointer<vtkProperty>::New();
    backProperty->SetColor(1.0, 0.0, 0.0);

    vtkSmartPointer<vtkParametricFunctionSource> 
    	parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->SetUResolution(51);
    parametricFunctionSource->SetVResolution(51);
    parametricFunctionSource->SetWResolution(51);
    parametricFunctionSource->Update();
    
    // mapper
    vtkSmartPointer<vtkPolyDataMapper>
    	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(
    	parametricFunctionSource->GetOutputPort());
 
    // actor
    vtkSmartPointer<vtkActor>
    	actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->SetBackfaceProperty(backProperty);

    // specify a transformation matrix
    Eigen::Matrix<float,4,4> T_model_world;
    T_model_world << 
	0, -1, 0, 0,
	1, 0, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1;

    // make vtk matrix
    auto T_model_world_vtk = 
	vtkSmartPointer<vtkMatrix4x4>::New();
    for(size_t i = 0; i < 4; ++i)
	for(size_t j = 0; j < 4; ++j)
	    T_model_world_vtk->SetElement(i,j,T_model_world(i,j));

    // Set up the transform filter
    vtkSmartPointer<vtkTransform> translation =
	vtkSmartPointer<vtkTransform>::New();
    translation->SetMatrix(T_model_world_vtk);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
	vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(parametricFunctionSource->GetOutputPort());
    transformFilter->SetTransform(translation);
    transformFilter->Update();

    // transformed mapper
    vtkSmartPointer<vtkPolyDataMapper>
    	transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    transformedMapper->SetInputConnection(
    	transformFilter->GetOutputPort());

    // transformed actor
    vtkSmartPointer<vtkActor> 
	transformedActor = vtkSmartPointer<vtkActor>::New();
    transformedActor->SetMapper(transformedMapper);
    transformedActor->GetProperty()->SetColor(0, 1, 0);
    transformedActor->GetProperty()->SetOpacity(0.2);
 
    // text mapper
    vtkSmartPointer<vtkTextMapper>
    	textmapper = vtkSmartPointer<vtkTextMapper>::New();
    textmapper->SetInput(parametricObject->GetClassName());
    textmapper->SetTextProperty(textProperty);
 
    // text actor
    vtkSmartPointer<vtkActor2D>
    	textactor = vtkSmartPointer<vtkActor2D>::New();
    textactor->SetMapper(textmapper);
    textactor->SetPosition(100, 16);

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
    renderer->AddActor(transformedActor);
    renderer->AddActor(textactor);
    renderer->SetBackground(0, 0, 0);
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(-30);
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
 
    renderWindow->Render();
    interactor->Start();
 
    return EXIT_SUCCESS;
}
