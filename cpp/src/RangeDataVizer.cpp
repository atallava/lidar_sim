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
#include <vtkPolygon.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkVersion.h>
#include <vtkCellArray.h>
#include <vtkExtractEdges.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkGraphicsFactory.h>
#include <vtkImagingFactory.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

RangeDataVizer::RangeDataVizer() :
    m_points_actor_server(),
    m_line_actor_server(),
    m_ellipsoid_actor_server(),
    m_brown_color{0.5451, 0.2706, 0.0645},
    m_bg_color{0, 0, 0},
    m_ellipsoid_skip(5)
{    
    
}

void RangeDataVizer::vizEllipsoidModels(const EllipsoidModels &ellipsoid_models)
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
	actors.push_back(
    	    m_ellipsoid_actor_server.genEllipsoidActor(ellipsoid_models[i].mu, 
    						       ellipsoid_models[i].cov_mat, ellipsoid_models[i].hit_prob));

    takeItAway(actors);
}

void RangeDataVizer::vizEllipsoidModels(const EllipsoidModels &ellipsoid_models, const Pts &pts)
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
    // actors.push_back(
    // 	m_ellipsoid_actor_server.genEllipsoidActor(origin_dot_mu, origin_dot_cov_mat));

    // ellipsoids
    for(size_t i = 0; i < n_ellipsoids; ++i)
    	actors.push_back(
    	    m_ellipsoid_actor_server.genEllipsoidActor(ellipsoid_models[i].mu, 
    						       ellipsoid_models[i].cov_mat, ellipsoid_models[i].hit_prob));

    // pts
    vtkSmartPointer<vtkActor> pts_actor = 
	m_points_actor_server.genPointsActor(pts);
    pts_actor->GetProperty()->SetColor(1, 1, 1);
    actors.push_back(pts_actor);
	
    takeItAway(actors);
}

void RangeDataVizer::takeItAway(const std::vector<vtkSmartPointer<vtkActor> > &actors)
{
    // renderer
    vtkSmartPointer<vtkRenderer>
    	renderer = vtkSmartPointer<vtkRenderer>::New();

    // todo: how to decide renderer size?
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
    renderer->SetBackground(m_bg_color[0], m_bg_color[1], m_bg_color[2]);
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

void RangeDataVizer::takeItAway(const vtkSmartPointer<vtkActor> &actor)
{
    std::vector<vtkSmartPointer<vtkActor> >actors;
    actors.push_back(actor);
    takeItAway(actors);
}

void RangeDataVizer::vizComparePts(const std::vector<std::vector<double> > &pts1,
				   const std::vector<std::vector<double> > &pts2)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;
    
    std::vector<double> color1 = {1, 1, 1};
    vtkSmartPointer<vtkActor> actor1 = m_points_actor_server.genPointsActor(pts1);
    actor1->GetProperty()->SetColor(color1[0], color1[1], color1[2]);
    actors.push_back(actor1);

    std::vector<double> color2 = {1, 0, 0};
    vtkSmartPointer<vtkActor> actor2 = m_points_actor_server.genPointsActor(pts2);
    actor2->GetProperty()->SetColor(color2[0], color2[1], color2[2]);
    actors.push_back(actor2);

    // fire up
    takeItAway(actors);
}

void RangeDataVizer::vizPts(const std::vector<std::vector<double> > &pts)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;
    actors.push_back(m_points_actor_server.genPointsActor(pts));

    // fire up
    takeItAway(actors);
}

void RangeDataVizer::vizTriangles(const Delaunay_cgal &triangulation, const std::vector<std::vector<double> > &pts)
{
    vtkSmartPointer<vtkActor> actor = m_triangles_actor_server.genTrianglesActor(triangulation, pts);
    takeItAway(actor);
}

void RangeDataVizer::vizTriangles(const std::vector<std::vector<int> > &triangle_vertex_ids, const std::vector<std::vector<double> > &pts)
{
    vtkSmartPointer<vtkActor> actor = m_triangles_actor_server.genTrianglesActor(triangle_vertex_ids, pts);
    takeItAway(actor);
}

void RangeDataVizer::vizSegmentation(const std::vector<std::vector<double> >& pts, const std::vector<int> &segmentation)
{
    std::vector<std::vector<double> > pts1 = logicalSubsetArray(pts, segmentation);
    std::vector<std::vector<double> > pts0 = logicalSubsetArray(pts, negateLogicalVec(segmentation));

    vizComparePts(pts1, pts0);
}

void RangeDataVizer::vizSectionModels(const SectionModelSim &sim)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    int ellipsoid_skip = 2;
    // ellipsoids
    for(size_t i = 0; i < sim.m_ellipsoid_model_sims.size(); ++i)
    {
    	EllipsoidModels this_ellipsoid_models = sim.m_ellipsoid_model_sims[i].m_ellipsoid_models;
    	for(size_t j = 0; j < this_ellipsoid_models.size(); j = j + ellipsoid_skip)
    	{
    	    EllipsoidModel this_ellipsoid_model = this_ellipsoid_models[j];
    	    // actors.push_back(m_ellipsoid_actor_server.genEllipsoidActor(this_ellipsoid_model.mu, this_ellipsoid_model.cov_mat, this_ellipsoid_model.hit_prob));
    	    actors.push_back(m_ellipsoid_actor_server.genEllipsoidActor(this_ellipsoid_model.mu, this_ellipsoid_model.cov_mat));
    	}
    }
			     
    // triangles
    for(size_t i = 0; i < sim.m_triangle_model_sims.size(); ++i)
    {
	TriangleModels this_triangle_models = sim.m_triangle_model_sims[i].m_triangle_models;
	std::vector<vtkSmartPointer<vtkActor> > tri_actors = 
	    m_triangles_actor_server.genTrianglesActors(this_triangle_models.m_triangles, 
							this_triangle_models.m_fit_pts, this_triangle_models.m_hit_prob_vec);

	actors.insert(actors.end(), tri_actors.begin(), tri_actors.end());
    }

    takeItAway(actors);
}

void RangeDataVizer::vizObjectMeshes(const std::vector<TriangleModelSim> &object_mesh_sims)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    for(size_t i = 0; i < object_mesh_sims.size(); ++i)
    {
	TriangleModels this_triangle_models = object_mesh_sims[i].m_triangle_models;
	vtkSmartPointer<vtkActor> actor = m_triangles_actor_server.genTrianglesActor(
	    this_triangle_models.m_triangles,
	    this_triangle_models.m_fit_pts);
										     
	actors.push_back(actor);
    }

    takeItAway(actors);
}

std::vector<vtkSmartPointer<vtkActor> >
RangeDataVizer::genSectionModelsActors(const SectionModelSim &sim)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    int ellipsoid_skip = 20;
    // ellipsoids
    for(size_t i = 0; i < sim.m_ellipsoid_model_sims.size(); ++i)
    {
    	EllipsoidModels this_ellipsoid_models = sim.m_ellipsoid_model_sims[i].m_ellipsoid_models;
    	for(size_t j = 0; j < this_ellipsoid_models.size(); j = j + ellipsoid_skip)
    	{
    	    EllipsoidModel this_ellipsoid_model = this_ellipsoid_models[j];
    	    actors.push_back(m_ellipsoid_actor_server.genEllipsoidActor(this_ellipsoid_model.mu, this_ellipsoid_model.cov_mat, this_ellipsoid_model.hit_prob));
    	}
    }
			     
    // triangles
    for(size_t i = 0; i < sim.m_triangle_model_sims.size(); ++i)
    {
    	TriangleModels this_triangle_models = sim.m_triangle_model_sims[i].m_triangle_models;
	vtkSmartPointer<vtkActor> tri_actor = 
	    m_triangles_actor_server.genTrianglesActor(this_triangle_models.m_triangles, this_triangle_models.m_fit_pts);
	tri_actor->GetProperty()->SetColor(0.5451, 0.2706, 0.0745);
	actors.push_back(tri_actor);
    }

    return actors;
}

std::vector<vtkSmartPointer<vtkActor> >
RangeDataVizer::genTriangleModelsActors(const std::vector<TriangleModelSim> &sims)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    // triangles
    for(size_t i = 0; i < sims.size(); ++i)
    {
	vtkSmartPointer<vtkActor> tri_actor = 
	    m_triangles_actor_server.genTrianglesActor(sims[i].m_triangle_models.m_triangles, 
						       sims[i].m_triangle_models.m_fit_pts);
	tri_actor->GetProperty()->SetColor(m_brown_color[0], m_brown_color[1], m_brown_color[2]);
	actors.push_back(tri_actor);
    }

    return actors;
}

std::vector<vtkSmartPointer<vtkActor> >
RangeDataVizer::genEllipsoidModelsActors(const std::vector<EllipsoidModelSim> &sims)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;

    // ellipsoids
    for(size_t i = 0; i < sims.size(); ++i)
    {
    	EllipsoidModels this_ellipsoid_models = sims[i].m_ellipsoid_models;
    	for(size_t j = 0; j < this_ellipsoid_models.size(); j = j + 1)
    	{
    	    EllipsoidModel this_ellipsoid_model = this_ellipsoid_models[j];
    	    actors.push_back(m_ellipsoid_actor_server.genEllipsoidActor(this_ellipsoid_model.mu, this_ellipsoid_model.cov_mat, this_ellipsoid_model.hit_prob));
    	}
    }
			     
    return actors;
}

std::vector<vtkSmartPointer<vtkActor> >
RangeDataVizer::genEllipsoidModelsActors(const EllipsoidModels &ellipsoid_models)
{
    std::vector<vtkSmartPointer<vtkActor> > actors;
    for(size_t i = 0; i < ellipsoid_models.size(); ++i)
    {
	EllipsoidModel ellipsoid_model = ellipsoid_models[i];
	actors.push_back(m_ellipsoid_actor_server.genEllipsoidActor(ellipsoid_model.mu, ellipsoid_model.cov_mat, ellipsoid_model.hit_prob));
    }

    return actors;
}

vtkSmartPointer<vtkActor> RangeDataVizer::genPointsActor(const std::vector<std::vector<double> > &points)
{
    return m_points_actor_server.genPointsActor(points);
}

vtkSmartPointer<vtkActor> RangeDataVizer::genPolyActor(const std::vector<std::vector<double> > &pts)
{
    // setup points
    vtkSmartPointer<vtkPoints> points =
	vtkSmartPointer<vtkPoints>::New();
    size_t pts_dim = pts[0].size();
    
    for(size_t i = 0; i < pts.size(); ++i)
	if (pts_dim == 2)
	    points->InsertNextPoint(pts[i][0], pts[i][1], 0);
	else
	    points->InsertNextPoint(pts[i][0], pts[i][1], pts[i][2]);

    // polygon
    vtkSmartPointer<vtkPolygon> polygon =
	vtkSmartPointer<vtkPolygon>::New();
    polygon->GetPointIds()->SetNumberOfIds(pts.size()); //make a quad
    for(size_t i = 0; i < pts.size(); ++i)
	polygon->GetPointIds()->SetId(i, i);

    // Add the polygon to a list of polygons
    vtkSmartPointer<vtkCellArray> polygons =
	vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(polygon);

    // Create a PolyData
    vtkSmartPointer<vtkPolyData> polygonPolyData =
	vtkSmartPointer<vtkPolyData>::New();
    polygonPolyData->SetPoints(points);
    polygonPolyData->SetPolys(polygons);
 
    // extract edges
    vtkSmartPointer<vtkExtractEdges> edges = 
	vtkSmartPointer<vtkExtractEdges>::New();
    edges->SetInput(polygonPolyData);

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
	vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInput(edges->GetOutput());
 
    vtkSmartPointer<vtkActor> actor =
	vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    return actor;
}

void RangeDataVizer::writeActorsToFile(const std::vector<vtkSmartPointer<vtkActor> > &actors,
				       const std::string rel_path_fig)
{
    // Setup offscreen rendering
    vtkSmartPointer<vtkGraphicsFactory> graphics_factory = 
	vtkSmartPointer<vtkGraphicsFactory>::New();
    graphics_factory->SetOffScreenOnlyMode( 1);
    graphics_factory->SetUseMesaClasses( 1 );
 
    vtkSmartPointer<vtkImagingFactory> imaging_factory = 
	vtkSmartPointer<vtkImagingFactory>::New();
    imaging_factory->SetUseMesaClasses( 1 ); 

    // renderer
      vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
    for(size_t i = 0; i < actors.size(); ++i)
	renderer->AddActor(actors[i]);
    renderer->ResetCamera();
    // renderer->GetActiveCamera()->Elevation(-90);
    renderer->GetActiveCamera()->Elevation(0);
    renderer->GetActiveCamera()->Zoom(1);
    renderer->ResetCameraClippingRange();

    // render window
    int renderer_size = 2000;
    vtkSmartPointer<vtkRenderWindow> renderWindow =
    	vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(
    	renderer_size, renderer_size);
    renderWindow->SetOffScreenRendering( 1 ); 
    renderWindow->AddRenderer(renderer);
    
    renderWindow->Render();

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

    // write to file
    writeRenderWindowToFile(renderWindow, rel_path_fig);
}

void RangeDataVizer::writeRenderWindowToFile(vtkSmartPointer<vtkRenderWindow> renderWindow, const std::string rel_path_fig)
{
    // write to file
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = 
	vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->Update();

    std::cout << "Writing image to file: " << rel_path_fig << std::endl;
 
    vtkSmartPointer<vtkPNGWriter> writer = 
	vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(rel_path_fig.c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
}
