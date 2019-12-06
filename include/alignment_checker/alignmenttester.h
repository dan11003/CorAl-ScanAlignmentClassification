#ifndef ALIGNMENTTESTER_H
#define ALIGNMENTTESTER_H
#include "random"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/scancomparsion.h"
#include "alignment_checker/utils.h"
namespace alignment_checker {


class AlignmentTester
{
public:

  AlignmentTester(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses);

  void PerformAndSaveTest(const std::string &dir);

  void ReAlignScansNoOffset();

  void ReAlignScansSmallOffset();


protected:

  Eigen::Affine3d vectorToAffine3d(const Eigen::Matrix<double,6,1> &v);

  double GausianNoiseGen(double dev);


  void AllocateScans();



  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses_;


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_small_terror_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > small_terror_;

  ros::Publisher pub, pub2;
  alignment_checker::VisComparsion vis_;

};

}
#endif // ALIGNMENTTESTER_H
