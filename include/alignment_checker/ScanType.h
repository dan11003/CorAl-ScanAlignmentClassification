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
#include "pcl/common/transforms.h"

//radar_mapping
#include "radar_mapping/radar_filters.h"
#include "radar_mapping/pointnormal.h"
#include "pcl/point_types.h"
#include "alignment_checker/Utils.h"
#include "radar_mapping/intensity_utils.h"

namespace CorAlignment{

using std::endl;
using std::cout;
using std::cerr;

typedef enum ScanType{none, rawlidar, rawradar, kstrong, kstrongCart, cfear, cen2018}scan_type;

std::string Scan2str(const scan_type& val);

scan_type Str2Scan(const std::string& val);
//represent all possible scan types here. e.g. polar radar e.g. polarRadarKstrong CartesianRadar etc.

class PoseScan
{
public:

  class Parameters
  {
  public:
    Parameters() {}
    std::string ToString(){
      std::ostringstream stringStream;
      stringStream << "scan_type, "<<Scan2str(scan_type)<<endl;
      stringStream << "sensor_min_distance, "<<sensor_min_distance<<endl;
      stringStream << "range_res, "<<range_res<<endl;
      stringStream << "kstrong, "<<kstrong<<endl;
      stringStream << "z_min, "<<z_min<<endl;
      stringStream << "compensate, "<<compensate<<endl;
      stringStream << "cart_resolution, "<<cart_resolution<<endl;
      stringStream << "cart_pixel_width, "<<cart_pixel_width<<endl;

      return stringStream.str();
    }

    ScanType scan_type = rawlidar;
    double sensor_min_distance = 2.5;



    //Radar
    double range_res = 0.04328;

    //kstrong
    int kstrong = 12;
    double z_min = 80;
    //CFEAR
    double resolution = 3;
    bool compensate = true;

    float cart_resolution = 0.2384;
    int cart_pixel_width = 300;



  };
  PoseScan(const PoseScan::Parameters pars, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);


  Eigen::Affine3d Test_;
  Eigen::Affine3d Tmotion_; // the motion
  const PoseScan::Parameters pars_;

  const Eigen::Affine3d& GetAffine() {return Test_;}

  virtual ~PoseScan() {}

  const std::string ToString(){return "PoseScan";}


  pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudCopy(const Eigen::Affine3d& T);

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudNoCopy() {return cloud_;}



  static int pose_count;

  const int pose_id;


protected:

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};


class RawRadar: public PoseScan{
public:

  RawRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

  const std::string ToString(){return "RawRadar";}

  cv_bridge::CvImagePtr polar_;
  double range_res_;
};


class CartesianRadar: public RawRadar{
public:

  CartesianRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

  const std::string ToString(){return "CartesianRadar";}

  cv_bridge::CvImagePtr polar_, polar_filtered_, cart_;
  double  sensor_min_distance, cart_resolution_;
  int cart_pixel_width_;


};

class kstrongRadar: public RawRadar
{
public:

  kstrongRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion );

  const std::string ToString(){return "kstrongRadar";}

};

class CFEARFeatures: public kstrongRadar
{
public:

  CFEARFeatures(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

  const std::string ToString(){return "CFEARFeatures";}

  radar_mapping::MapNormalPtr CFEARFeatures_;
};

class RawLidar: public PoseScan{
public:

  RawLidar(const PoseScan::Parameters pars, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion ) : PoseScan(pars,T,Tmotion){ cloud_ = cloud; }

  const std::string ToString(){return "RawLidar";}

};

typedef std::shared_ptr<PoseScan> PoseScan_S;

PoseScan_S RadarPoseScanFactory(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr radar_msg, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);
}

