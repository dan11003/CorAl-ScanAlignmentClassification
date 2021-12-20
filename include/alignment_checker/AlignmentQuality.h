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
#include "alignment_checker/ScanType.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/flann.h"
#include "pcl/kdtree/kdtree.h"
#include "memory.h"
#include <iomanip>
#include "radar_mapping/n_scan_normal.h"

// ROS tf
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "alignment_checker/Utils.h"



namespace CorAlignment{
using std::endl;
using std::cout;
using std::cerr;
namespace ac = alignment_checker;


///////////// BASE /////////////////

//std::map<int,std::string> QualityTypes{{0,"CorAl"},{}}

class AlignmentQuality
{
public:
  class parameters
  {
  public:
    parameters() {}
    std::string method;
    scan_type scantype;
    double radius;

    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "OdometryKeyframeFuser::Parameters"<<endl;
      stringStream << "method,"<<method<<endl;
      stringStream << "radius,"<<radius<<endl;
      return stringStream.str();
    }
  };

  AlignmentQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : par_(par), Toffset_(Toffset) {
    src_ = src;
    ref_ = ref;
    quality_ = {0,0,0};
  }

  void Visualize();

  virtual ~AlignmentQuality(){}

  virtual std::vector<double> GetResiduals() = 0;

  virtual std::vector<double> GetQualityMeasure() = 0;

  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);

  std::shared_ptr<PoseScan> src_, ref_;
  AlignmentQuality::parameters par_;
  const Eigen::Affine3d Toffset_;
  std::vector<double> quality_;
  std::vector<double> residuals_;



};
typedef std::shared_ptr<AlignmentQuality> AlignmentQuality_S;

/************** P2P ********************/

class p2pQuality: public AlignmentQuality
{
public:

  p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

  ~p2pQuality(){}

  std::vector<double> GetResiduals() {
    return residuals_;
    //return {0,0,0};
  }

  std::vector<double> GetQualityMeasure();

protected:
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

/************** P2D ********************/

class p2dQuality: public AlignmentQuality
{
public:

  p2dQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : AlignmentQuality(src, ref, par, Toffset){cout<<"p2d quality"<<endl;}

  ~p2dQuality(){}

  std::vector<double> GetResiduals(){return {0,0,0}; }

  std::vector<double> GetQualityMeasure();
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};
/************** P2D ********************/



/************** CorAl *******************/

class CorAl: public AlignmentQuality
{
public:

  CorAl(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity())  : AlignmentQuality(src, ref, par, Toffset){}

  ~CorAl(){}

  std::vector<double> GetResiduals(){ return {0,0,0}; }

  std::vector<double> GetQualityMeasure();
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

class CFEARQuality: public AlignmentQuality
{
public:

  CFEARQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

  ~CFEARQuality(){}

  std::vector<double> GetResiduals(){ return {0,0,0}; }

  std::vector<double> GetQualityMeasure(){return {0,0,0};}


  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

class CorAlRadarQuality: public AlignmentQuality
{
public:

  CorAlRadarQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

  ~CorAlRadarQuality(){}

  std::vector<double> GetResiduals(){ return {0,0,0}; }

  std::vector<double> GetQualityMeasure(){return {0,0,0};}

protected:

  void GetNearby(const pcl::PointXY& query, Eigen::MatrixXd& nearby_src, Eigen::MatrixXd& nearby_ref, Eigen::MatrixXd& merged);

  bool Covariance(Eigen::MatrixXd& x, Eigen::Matrix2d& cov);

  pcl::PointCloud<pcl::PointXY>::Ptr ref_pcd, src_pcd;
  std::vector<double> ref_i, src_i ;
  pcl::KdTreeFLANN<pcl::PointXY> kd_src, kd_ref;

  std::vector<double> sep_res_;
  std::vector<bool> sep_valid;

  std::vector<double> joint_res_;





  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};








class AlignmentQualityFactory
{
public:
  static AlignmentQuality_S CreateQualityType(std::shared_ptr<PoseScan>& ref, std::shared_ptr<PoseScan>& src,  const AlignmentQuality::parameters& pars, const Eigen::Affine3d& Toffset = Eigen::Affine3d::Identity()) {
    AlignmentQuality_S quality = nullptr;

    // CFEAR FEATURES + ANY SCORE (P2P/P2L/P2D)
    if(std::dynamic_pointer_cast<CFEARFeatures>(ref)!=nullptr && std::dynamic_pointer_cast<CFEARFeatures>(src)!=nullptr){
      quality = std::make_shared<CFEARQuality>(CFEARQuality(ref,src,pars,Toffset));
    }// RAW LIDAR (P2P/P2D/CORAL)
    else if(std::dynamic_pointer_cast<RawLidar>(ref)!=nullptr && std::dynamic_pointer_cast<RawLidar>(src)!=nullptr){
      if(pars.method=="coral")
        quality = std::make_shared<CorAl>(CorAl(ref,src,pars,Toffset));
      else if(pars.method=="P2D")
        quality = AlignmentQuality_S(new p2dQuality(ref,src,pars,Toffset));
      else if(pars.method=="P2P")
        quality = AlignmentQuality_S(new p2pQuality(ref,src,pars,Toffset));
    }

    assert(quality != nullptr);
    return quality;
  }
};

class AlignmentQualityPlot
{
public:
  AlignmentQualityPlot() {}


  //Modify to poseScan instead of MapNormalPtr
  static void PublishPoseScan(const std::string& topic, std::shared_ptr<PoseScan>& p_scan, const Eigen::Affine3d& T, const std::string& frame_id, const int value=0);

  //static std::map<std::string, ros::Publisher> pubs;
};





}
