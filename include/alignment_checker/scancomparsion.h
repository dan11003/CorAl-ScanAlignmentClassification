#ifndef SCANCOMPARISON_H
#define SCANCOMPARISON_H
#include "alignment_checker/scan.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
namespace alignment_checker {

using std::endl;
using std::cout;
using std::cerr;

class ScanComparsion
{
public:

  ScanComparsion( const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target);

  bool CheckAlignment(double tolerance=0.3);

  void CalculateOverlap();

  void SetNext(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetSrc(){return src_;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetTar(){return target_;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetSrcOverlap(){return src_overlap_;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetTarOverlap(){return tar_overlap_;}



protected:


  pcl::PointCloud<pcl::PointXYZ>::Ptr src_, target_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_overlap_, src_overlap_;
  boost::shared_ptr<ScanType> ScanSrc_, ScanTarget_;
  double radius_ = 0.5;

};

typedef boost::shared_ptr<ScanComparsion> boostScan;

class VisComparsion
{
public:

  VisComparsion(const std::string frameid="/world");

  void PlotClouds(boost::shared_ptr<ScanComparsion> &comp);

private:

  pcl::PointCloud<pcl::PointXYZ>::Ptr Stamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time &tcloud);

  ros::NodeHandle nh_;
  ros::Publisher pub_src, pub_tar, pub_overlap_src, pub_overlap_tar;
  std::string frameid_;
};

}
#endif // SCANCOMPARISON_H
