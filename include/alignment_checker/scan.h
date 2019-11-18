#ifndef POINTSCORE_H
#define POINTSCORE_H

#include "stdio.h"
#include "iostream"
#include <vector>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <Eigen/Eigenvalues>
#include "Eigen/Dense"

#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>
#include "pcl/common/centroid.h"
#include "math.h"
namespace alignment_checker{
using std::cout;
using std::cerr;
using std::endl;
class ScanType
{
public:

  ScanType();

  ScanType(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);


  virtual void SetInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);

  virtual double GetInformation(const double r=0.6);

  void GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double r, std::vector<int> &idx ){}

  void GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double r, std::vector<int> &idx_input, std::vector<int> &idx_this );

  //!
  //! \brief GetOverlap
  //! \return points in the scan which overlap with the input
  //!
  virtual pcl::PointCloud<pcl::PointXYZ>::Ptr GetOverlap(const ScanType &scan){return NULL;}

  void GetOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_src, double r=0.5);

  void ExtractIndecies(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, std::vector<int> &indecies, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);



protected:

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;


};



}
#endif // POINTSCORE_H
