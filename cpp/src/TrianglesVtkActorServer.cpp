#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

#include <vtkSmartPointer.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkCellArray.h>
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
#include <vtkExtractEdges.h>
#include <vtkTriangle.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <lidar_sim/TrianglesVtkActorServer.h>

using namespace lidar_sim;

TrianglesVtkActorServer::TrianglesVtkActorServer()
{
}

vtkSmartPointer<vtkActor> TrianglesVtkActorServer::genTrianglesActor(const Delaunay_cgal &triangulation,
								     const std::vector<std::vector<double> > &m_fit_pts)
{
    // insert points
    vtkSmartPointer<vtkPoints> points =
	vtkSmartPointer<vtkPoints>::New();
    for(size_t i = 0; i < m_fit_pts.size(); ++i)
	points->InsertNextPoint(m_fit_pts[i][0], m_fit_pts[i][1], m_fit_pts[i][2]);

    // insert triangles
    vtkSmartPointer<vtkCellArray> triangles =
	vtkSmartPointer<vtkCellArray>::New();
    for(Delaunay_cgal::Finite_faces_iterator fit = triangulation.finite_faces_begin();
	fit != triangulation.finite_faces_end(); ++fit) 
    {
	Delaunay_cgal::Face_handle face = fit;
	
	vtkSmartPointer<vtkTriangle> triangle =
	    vtkSmartPointer<vtkTriangle>::New();
	for(size_t i = 0; i < 3; ++i)
	    triangle->GetPointIds()->SetId (i, face->vertex(i)->info());
	
	triangles->InsertNextCell(triangle);
    }

    // polydata
    vtkSmartPointer<vtkPolyData> polyData =
	vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
 
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

    return actor;
}
