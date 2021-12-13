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


namespace CorAlignment{
using std::endl;
using std::cout;
using std::cerr;


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
    double resolution;

    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "OdometryKeyframeFuser::Parameters"<<endl;
      stringStream << "method, "<<method<<endl;
      stringStream << "resolution, "<<resolution<<endl;
      return stringStream.str();
    }
  };

  AlignmentQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : src_(src_), ref_(ref), par_(par), Toffset_(Toffset) {}

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

/************** P2D ********************/

class p2pQuality: public AlignmentQuality
{
public:

  p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

  ~p2pQuality(){}

  std::vector<double> GetResiduals() {return {0,0,0}; }

  std::vector<double> GetQualityMeasure();

protected:
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

/************** P2D ********************/

class p2dQuality: public AlignmentQuality
{
public:

  p2dQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

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



class AlignmentQualityFactory
{
public:
  static std::unique_ptr<AlignmentQuality> CreateQualityType(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& pars, const Eigen::Affine3d& Toffset = Eigen::Affine3d::Identity()) {
    if(pars.method=="coral")
      return std::make_unique<CorAl>(CorAl(ref,src,pars,Toffset));
    else if(pars.method=="p2d")
      return std::make_unique<p2dQuality>(p2dQuality(ref,src,pars,Toffset));
    else if(pars.method=="p2p")
      return std::make_unique<p2pQuality>(p2pQuality(ref,src,pars,Toffset));
    else return nullptr;
  }
};







}
