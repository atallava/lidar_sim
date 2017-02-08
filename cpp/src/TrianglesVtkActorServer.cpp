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

TrianglesVtkActorServer::TrianglesVtkActorServer() :
    m_max_opacity(0.9),
    m_brown_color{0.5451, 0.2706, 0.0645}
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

vtkSmartPointer<vtkActor> 
TrianglesVtkActorServer::genTrianglesActor(const std::vector<std::vector<int> > &triangle_vertex_ids, 
					    const std::vector<std::vector<double> > &pts)
{
    // insert points
    vtkSmartPointer<vtkPoints> points =
	vtkSmartPointer<vtkPoints>::New();
    for(size_t i = 0; i < pts.size(); ++i)
	points->InsertNextPoint(pts[i][0], pts[i][1], pts[i][2]);

    // insert triangles
    vtkSmartPointer<vtkCellArray> triangles =
	vtkSmartPointer<vtkCellArray>::New();
    for(size_t i = 0; i < triangle_vertex_ids.size(); ++i)
    {
	vtkSmartPointer<vtkTriangle> triangle =
	    vtkSmartPointer<vtkTriangle>::New();
	for(size_t j = 0; j < 3; ++j)
	    triangle->GetPointIds()->SetId (j, triangle_vertex_ids[i][j]);
	
	triangles->InsertNextCell(triangle);
    }

    // polydata
    vtkSmartPointer<vtkPolyData> polyData =
	vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
 
    // extract edges
    // vtkSmartPointer<vtkExtractEdges> edges = 
    // 	vtkSmartPointer<vtkExtractEdges>::New();
    // edges->SetInput(polyData);

    // mapper
    vtkSmartPointer<vtkPolyDataMapper>
    	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // mapper->SetInput(edges->GetOutput());
    mapper->SetInput(polyData);

    // actor
    vtkSmartPointer<vtkActor>
    	actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    return actor;
}

std::vector<vtkSmartPointer<vtkActor> > 
TrianglesVtkActorServer::genTrianglesActors(const std::vector<std::vector<int> > &triangle_vertex_ids, 
					    const std::vector<std::vector<double> > &pts, const std::vector<double> hit_prob_vec)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    // each triangle is its own actor
    for(size_t i = 0; i < triangle_vertex_ids.size(); ++i)
    {
	vtkSmartPointer<vtkPoints> points =
	    vtkSmartPointer<vtkPoints>::New();
	for(size_t j = 0; j < 3; ++j)
	{
	    int v_id = triangle_vertex_ids[i][j];
	    points->InsertNextPoint(pts[v_id][0], pts[v_id][1], pts[v_id][2]);
	}

	vtkSmartPointer<vtkTriangle> triangle =
	    vtkSmartPointer<vtkTriangle>::New();
	
	for(size_t j = 0; j < 3; ++j)
	    triangle->GetPointIds()->SetId (j, j);

	vtkSmartPointer<vtkCellArray> triangles =
	    vtkSmartPointer<vtkCellArray>::New();
	triangles->InsertNextCell(triangle);

	// polydata
	vtkSmartPointer<vtkPolyData> polyData =
	    vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);
	polyData->SetPolys(triangles);

	// mapper
	vtkSmartPointer<vtkPolyDataMapper>
	    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInput(polyData);

	// actor
	vtkSmartPointer<vtkActor>
	    actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// set color and opacity
	actor->GetProperty()->SetColor(m_brown_color[0], m_brown_color[1], m_brown_color[2]);
	actor->GetProperty()->SetOpacity(mapHitProbToOpacity(hit_prob_vec[i]));

	actors.push_back(actor);
    }

    return actors;
}							      

std::vector<vtkSmartPointer<vtkActor> > 
TrianglesVtkActorServer::genTrianglesActors(const std::vector<std::vector<int> > &triangle_vertex_ids, 
					    const std::vector<std::vector<double> > &pts)
{
    // todo: how to make this default?
    std::vector<double> hit_prob_vec(triangle_vertex_ids.size(), 1);
    return genTrianglesActors(triangle_vertex_ids, pts, hit_prob_vec);
}

double TrianglesVtkActorServer::mapHitProbToOpacity(double hit_prob)
{
    // linear scale
    return hit_prob*m_max_opacity;
}

