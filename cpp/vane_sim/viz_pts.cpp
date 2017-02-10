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
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>

using namespace lidar_sim;
 
int main(int, char *[])
{
    std::string rel_path_pts = "data/sections/section_03/section_03_block_02_ground_triangles_fit_pts.txt";

    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    RangeDataVizer vizer;
    vizer.vizPts(pts);

    return EXIT_SUCCESS;
}
