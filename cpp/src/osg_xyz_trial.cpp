#include <string>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Vec3>

#include <osgViewer/Viewer>

#include <lidar_sim/Utils.h>

using namespace lidar_sim;

int main() {
    osg::Group* root = new osg::Group();
    std::string rel_path_xyz = "../data/taylorJune2014/sections/world_frame/section_pts_01_world_frame_subsampled.xyz";
    root->addChild(osgXYZLoader(rel_path_xyz));

    osgViewer::Viewer viewer;

    viewer.setSceneData(root);

    viewer.run();

    return 0;
}
