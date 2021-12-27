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
      stringStream << "range_min, "<<range_min<<endl;
      stringStream << "range_res, "<<range_res<<endl;
      stringStream << "kstrong, "<<kstrong<<endl;
      stringStream << "z_min, "<<z_min<<endl;
      return stringStream.str();
    }

    ScanType scan_type = rawlidar;

    //generic parameters
    double range_min = 0;

    //Radar
    double range_res = 0.04328;

    //kstrong
    int kstrong = 12;
    double z_min = 60;
    //CFEAR
    double resolution = 3;



  };
  PoseScan(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot) : Test_(T), Tmot_(Tmot), cloud_(new pcl::PointCloud<pcl::PointXYZI>()),pose_id(pose_count++){cout<<"Created posescan"<<endl;}
  Eigen::Affine3d Test_, Tmot_;

  const Eigen::Affine3d& GetAffine() {return Test_;}

  virtual ~PoseScan() {}

  const std::string ToString(){return "PoseScan";}

  static int pose_count;

  const int pose_id;

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudCopy(const Eigen::Affine3d& T);

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudNoCopy() {return cloud_;}

protected:

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

class RawRadar: public PoseScan{
public:

  RawRadar(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot ) : PoseScan(T,Tmot){ polar_ = polar;}

  const std::string ToString(){return "RawRadar";}

  cv_bridge::CvImagePtr polar_;
};


class CartesianRadar: public RawRadar{
public:

  CartesianRadar(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, int kstrong = 12, double z_min = 60, double range_res = 0.04328, double min_distance = 2.5);

  const std::string ToString(){return "CartesianRadar";}

  cv_bridge::CvImagePtr polar_, polar_filtered_, cart_;


};

class kstrongRadar: public RawRadar
{
public:

  kstrongRadar(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot , int kstrong = 12, double z_min = 60, double range_res = 0.04328, double min_distance = 2.5 );

  const std::string ToString(){return "kstrongRadar";}

};

class CFEARFeatures: public kstrongRadar
{
public:

  CFEARFeatures(cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot , int kstrong = 12, double z_min = 60, double range_res = 0.04328, double min_distance = 2.5, double resolution = 3 );

  const std::string ToString(){return "CFEARFeatures";}

  radar_mapping::MapNormalPtr CFEARFeatures_;
};

class RawLidar: public PoseScan{
public:

  RawLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot ) : PoseScan(T,Tmot){ cloud_ = cloud; }

  const std::string ToString(){return "RawLidar";}

};

typedef std::shared_ptr<PoseScan> PoseScan_S;


}

