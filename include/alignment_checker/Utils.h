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
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/registration/transforms.h"
#include "pcl/common/distances.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
namespace alignment_checker {

void SetScanLocations(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds,  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses);

void SetScanLocation(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud,  Eigen::Affine3d &pose);

void ReadCloudsFromFile(const std::string directory, const std::string prefix, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, int start_index=0);

void ReadPosesFromFile(const std::string &filepath, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses);

Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v);

Eigen::Affine3d VectorToAffine3d(const Eigen::Matrix<double, 6,1> &v);

Eigen::Affine3d VectorToAffine3dxyez(double x, double y, double theta);

Eigen::Affine3d VectorToAffine3dxyez(const std::vector<double>& vek);

void SegmentGround(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &filtered, double height);

void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, pcl::PointCloud<pcl::Normal>::Ptr  &normal,float voxelsize=0.1);

void FilterClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &normals);

void FilterCloudsByDistance(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses, double radius);

pcl::PointCloud<pcl::PointXY>::Ptr pcl3dto2d(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, std::vector<double>& intensity);


template <typename T>
std::ostream & operator << (std::ostream & os, const std::vector<T> & vec)
{
    for(auto elem : vec)
    {
        os<<elem<< " ";
    }
    return os;
}

}
#endif // IO_H
