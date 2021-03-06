#pragma once
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

namespace lidar_sim {
    class Visualizer {
    public:
    Visualizer() : write_to_file_(false), file_name_("") {}
        void visualize(std::string file_name, int is_rgb);
        void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1,
		       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2);
	void setFileLocation(std::string file_name);
	void disableWriteToFile();

    private:
	bool write_to_file_;
	std::string file_name_;
    };
}
