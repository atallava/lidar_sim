#include <pcl/common/common_headers.h>
#include <lidar_sim/Visualizer.h>

using namespace lidar_sim;

void Visualizer::visualize(std::string file_name)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_name, *cloud);
    visualize(cloud);
}

void Visualizer::visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer("viz");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
    viewer.addCoordinateSystem(1.0);
    // am scene
    //viewer.setCameraPosition(47.0035, 347.314, 71.6445, 333.539, 79.7573, -24.0705, 0.119185, -0.218815, 0.96846);
    // an scene
    viewer.setCameraPosition(11.1799, 153.713, 18.8741, 333.539, 79.7573, -24.0705, 0.0946536, -0.158154, 0.982867);
    
    if (write_to_file_)
	viewer.saveScreenshot(file_name_);

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }     
}

void Visualizer::visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1,
			   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2)
{
  pcl::visualization::PCLVisualizer viewer("viz");
  viewer.initCameraParameters();

  int v1(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.addText("Cloud 1", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_1(cloud_1);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud_1, rgb_1, "cloud_v1", v1);
  viewer.addPointCloud(cloud_1, rgb_1, "cloud_1", v1);
  
  int v2(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.addText("Cloud 2", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_2(cloud_2);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud_2, rgb_2, "cloud_v2", v2);
  viewer.addPointCloud(cloud_2, rgb_2, "cloud_2", v2);

  viewer.addCoordinateSystem(1.0);
  // an scene
  viewer.setCameraPosition(11.1799, 153.713, 18.8741, 333.539, 79.7573, -24.0705, 0.0946536, -0.158154, 0.982867);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }
}

void Visualizer::setFileLocation(std::string file_name)
{
    file_name_.assign(file_name);
    write_to_file_ = true;
}

void Visualizer::disableWriteToFile()
{
    write_to_file_ = false;
}
