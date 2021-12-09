#pragma once


#include "memory.h"
//Eigen
#include "Eigen/Dense"

//PCL
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"

// OpenCv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace CorAlignment{


class PoseScan
{
public:
  PoseScan(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot) : Test_(T), Tmot_(Tmot){}
  Eigen::Affine3d Test_, Tmot_;
};

class radarscan: public PoseScan{
public:
  radarscan(const cv::Mat& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot );
  cv::Mat polar_;

};

class lidarscan: public PoseScan
{
public:
  lidarscan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot );
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

};

}

