#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include <vtkExtractEdges.h>

#include <lidar_sim/RangeDataVizer.h>

int main(int, char *[])
{ 
    // Create a triangle
    vtkSmartPointer<vtkPoints> points =
	vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint ( 1.0, 0.0, 0.0 );
    points->InsertNextPoint ( 0.0, 0.0, 0.0 );
    points->InsertNextPoint ( 0.0, 1.0, 0.0 );
    points->InsertNextPoint ( 0.0, -1.0, 0.0 );
 
    vtkSmartPointer<vtkTriangle> triangle1 =
	vtkSmartPointer<vtkTriangle>::New();
    triangle1->GetPointIds()->SetId ( 0, 0 );
    triangle1->GetPointIds()->SetId ( 1, 1 );
    triangle1->GetPointIds()->SetId ( 2, 2 );
 
    vtkSmartPointer<vtkTriangle> triangle2 =
	vtkSmartPointer<vtkTriangle>::New();
    triangle2->GetPointIds()->SetId ( 0, 0 );
    triangle2->GetPointIds()->SetId ( 1, 1 );
    triangle2->GetPointIds()->SetId ( 2, 3 );
 
    vtkSmartPointer<vtkCellArray> triangles =
	vtkSmartPointer<vtkCellArray>::New();
    triangles->InsertNextCell ( triangle1 );
    triangles->InsertNextCell ( triangle2 );
 
    // Create a polydata object
    vtkSmartPointer<vtkPolyData> polyData =
	vtkSmartPointer<vtkPolyData>::New();
 
    // Add the geometry and topology to the polydata
    polyData->SetPoints ( points );
    polyData->SetPolys ( triangles );
 
    // extract edges
    vtkSmartPointer<vtkExtractEdges> edges = 
	vtkSmartPointer<vtkExtractEdges>::New();
    edges->SetInput(polyData);

    // mapper
    vtkSmartPointer<vtkPolyDataMapper>
    	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInput(edges->GetOutput());
    
    // actor
    vtkSmartPointer<vtkActor>
    	actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    lidar_sim::RangeDataVizer vizer;
    vizer.takeItAway(actor);
	
    return(0);
}
