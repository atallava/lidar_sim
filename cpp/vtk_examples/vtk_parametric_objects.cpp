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
 
#include <vtkParametricTorus.h>
#include <vtkParametricBoy.h>
#include <vtkParametricConicSpiral.h>
#include <vtkParametricCrossCap.h>
#include <vtkParametricDini.h>
#include <vtkParametricEllipsoid.h>
#include <vtkParametricEnneper.h>
#include <vtkParametricFigure8Klein.h>
#include <vtkParametricKlein.h>
#include <vtkParametricMobius.h>
#include <vtkParametricRandomHills.h>
#include <vtkParametricRoman.h>
#include <vtkParametricSpline.h>
#include <vtkParametricSuperEllipsoid.h>
#include <vtkParametricSuperToroid.h>
#include <vtkParametricTorus.h>
 
#include <vector>
 
int main(int, char *[])
{
    // Select one of the following (matching the selection above)
    vtkSmartPointer<vtkParametricFunction> 
	parametricObject = vtkSmartPointer<vtkParametricEllipsoid>::New();
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetXRadius(.5);
    static_cast<vtkParametricEllipsoid *>(
	parametricObject.GetPointer())->SetYRadius(2.0);
 
    // vtkSmartPointer<vtkParametricFunctionSource> 
    // 	parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    // vtkSmartPointer<vtkRenderer>
    // 	renderer;
    // vtkSmartPointer<vtkPolyDataMapper>
    // 	mapper;
    // vtkSmartPointer<vtkActor>
    // 	actor;
    // vtkSmartPointer<vtkTextMapper>
    // 	textmapper;
    // vtkSmartPointer<vtkActor2D>
    // 	textactor;
 
    // // Create one text property for all
    // vtkSmartPointer<vtkTextProperty> textProperty =
    // 	vtkSmartPointer<vtkTextProperty>::New();
    // textProperty->SetFontSize(10);
    // textProperty->SetJustificationToCentered();
 
    // vtkSmartPointer<vtkProperty> backProperty =
    // 	vtkSmartPointer<vtkProperty>::New();
    // backProperty->SetColor(1.0, 0.0, 0.0);

    // parametricFunctionSource->SetParametricFunction(parametricObjects);
    // parametricFunctionSource->SetUResolution(51);
    // parametricFunctionSource->SetVResolution(51);
    // parametricFunctionSource->SetWResolution(51);
    // parametricFunctionSource->Update();
 
    // mapper.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
    // mapper[i]->SetInputConnection(
    // 	parametricFunctionSource[i]->GetOutputPort());
 
    // actor.push_back(vtkSmartPointer<vtkActor>::New());
    // actor[i]->SetMapper(mapper[i]);
    // actor[i]->SetBackfaceProperty(backProperty);
 
    // textmapper.push_back(vtkSmartPointer<vtkTextMapper>::New());
    // textmapper[i]->SetInput(parametricObjects[i]->GetClassName());
    // textmapper[i]->SetTextProperty(textProperty);
 
    // textactor.push_back(vtkSmartPointer<vtkActor2D>::New());
    // textactor[i]->SetMapper(textmapper[i]);
    // textactor[i]->SetPosition(100, 16);
    // renderer.push_back(vtkSmartPointer<vtkRenderer>::New());

    // unsigned int gridDimensions = 4;
 
    // // Need a renderer even if there is no actor
    // for(size_t i = parametricObjects.size();
    // 	i < gridDimensions * gridDimensions;
    // 	i++)
    // {
    // 	renderer.push_back(vtkSmartPointer<vtkRenderer>::New());
    // }
 
    // vtkSmartPointer<vtkRenderWindow> renderWindow =
    // 	vtkSmartPointer<vtkRenderWindow>::New();
    // int rendererize = 200;
    // renderWindow->SetSize(
    // 	rendererize*gridDimensions, rendererize*gridDimensions);
 
    // for(int row = 0; row < static_cast<int>(gridDimensions); row++)
    // {
    // 	for(int col = 0; col < static_cast<int>(gridDimensions); col++)
    // 	{
    // 	    int index = row*gridDimensions + col;
 
    // 	    // (xmin, ymin, xmax, ymax)
    // 	    double viewport[4] =
    // 		{static_cast<double>(col) * rendererize / (gridDimensions * rendererize),
    // 		 static_cast<double>(gridDimensions - (row+1)) * rendererize / (gridDimensions * rendererize),
    // 		 static_cast<double>(col+1)*rendererize / (gridDimensions * rendererize),
    // 		 static_cast<double>(gridDimensions - row) * rendererize / (gridDimensions * rendererize)};
 
    // 	    renderWindow->AddRenderer(renderer[index]);
    // 	    renderer[index]->SetViewport(viewport);
    // 	    if(index > static_cast<int>(parametricObjects.size() - 1))
    // 	    {
    // 		continue;
    // 	    }
    // 	    renderer[index]->AddActor(actor[index]);
    // 	    renderer[index]->AddActor(textactor[index]);
    // 	    renderer[index]->SetBackground(.2, .3, .4);
    // 	    renderer[index]->ResetCamera();
    // 	    renderer[index]->GetActiveCamera()->Azimuth(30);
    // 	    renderer[index]->GetActiveCamera()->Elevation(-30);
    // 	    renderer[index]->GetActiveCamera()->Zoom(0.9);
    // 	    renderer[index]->ResetCameraClippingRange();
    // 	}
    // }
 
    // vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    // 	vtkSmartPointer<vtkRenderWindowInteractor>::New();
    // interactor->SetRenderWindow(renderWindow);
 
    // renderWindow->Render();
    // interactor->Start();
 
    return EXIT_SUCCESS;
}
