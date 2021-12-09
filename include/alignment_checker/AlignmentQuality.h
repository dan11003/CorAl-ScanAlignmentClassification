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

namespace CorAlignment{


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
  };

  AlignmentQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : src_(src_), ref_(ref), par_(par), Toffset_(Toffset) {}

  virtual std::vector<double> GetResiduals() = 0;

  virtual std::vector<double> GetQualityMeasure() = 0;
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);

  std::shared_ptr<PoseScan> src_, ref_;
  AlignmentQuality::parameters par_;
  const Eigen::Affine3d Toffset_;

};


/************** P2D ********************/

class p2dQuality: public AlignmentQuality
{
public:

  p2dQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : AlignmentQuality(src, ref, par, Toffset){}

  std::vector<double> GetResiduals(){return {0,0,0}; }

  std::vector<double> GetQualityMeasure();
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);

  std::shared_ptr<PoseScan> src_, ref_;

};
/************** P2D ********************/



/************** CorAl *******************/

class CorAl: public AlignmentQuality
{
public:

  CorAl(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity())  : AlignmentQuality(src, ref, par, Toffset){}

  std::vector<double> GetResiduals(){ return {0,0,0}; }

  std::vector<double> GetQualityMeasure();
  // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);

  std::shared_ptr<PoseScan> src_, ref_;



};



class AlignmentQualityFactory
{
public:
  static std::unique_ptr<AlignmentQuality> CreateQualityType(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& pars, const Eigen::Affine3d& Toffset = Eigen::Affine3d::Identity()) {
    if(pars.method=="coral")
      return std::make_unique<CorAl>(CorAl(ref,src,pars,Toffset));
    else if(pars.method=="p2d")
      return std::make_unique<p2dQuality>(p2dQuality(ref,src,pars,Toffset));
    else return nullptr;
  }
};







}
