#ifndef IO_H
#define IO_H
#include <Eigen/Eigen>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "iostream"
#include "ros/ros.h"
#include <fstream>
#include <pcl/io/pcd_io.h>
#include "pcl/filters/voxel_grid.h"
using std::cout;
using std::cerr;
using std::endl;
using std::string;
namespace alignment_checker {

void ReadCloudsFromFile(const std::string directory, const std::string prefix, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds);

void ReadPosesFromFile(const std::string &filepath, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses);

Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v);

Eigen::Affine3d VectorToAffine3d(const Eigen::Matrix<double, 6,1> &v);

void SegmentGround(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &filtered, double height);

void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, pcl::PointCloud<pcl::Normal>::Ptr  &normal,float voxelsize=0.1);

void FilterClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &normals);

}
#endif // IO_H
