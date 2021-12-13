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

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace CorAlignment{


class PoseScan
{
public:
  PoseScan(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot) : Test_(T), Tmot_(Tmot){}
  Eigen::Affine3d Test_, Tmot_;

  const Eigen::Affine3d& GetAffine() {return Test_;}

  virtual ~PoseScan() {}

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudCopy(const Eigen::Affine3d& T);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudNoCopy() {return cloud_;}
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

};

typedef std::shared_ptr<PoseScan> PoseScan_S;

}

