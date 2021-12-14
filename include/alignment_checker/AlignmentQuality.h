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
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
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



class AlignmentQualityFactory
{
public:
  static AlignmentQuality_S CreateQualityType(std::shared_ptr<PoseScan>& ref, std::shared_ptr<PoseScan>& src,  const AlignmentQuality::parameters& pars, const Eigen::Affine3d& Toffset = Eigen::Affine3d::Identity()) {
    AlignmentQuality_S quality = nullptr;
    cout<<"Create quality type"<<std::quoted(pars.method)<<endl;
    if(pars.method=="coral")
      quality = std::make_shared<CorAl>(CorAl(ref,src,pars,Toffset));
    else if(pars.method=="p2d"){
      quality = AlignmentQuality_S(new p2dQuality(ref,src,pars,Toffset));
    }
    else if(pars.method=="p2p")
      quality = std::make_shared<p2pQuality>(p2pQuality(ref,src,pars,Toffset));
    assert(quality != nullptr);
    return quality;
  }
};

class AlignmentQualityPlot
{
public:
  AlignmentQualityPlot() {}


  //Modify to poseScan instead of MapNormalPtr
  //static void PublishMap(const std::string& topic, MapNormalPtr map, Eigen::Affine3d& T, const std::string& frame_id, const int value=0);

  //static std::map<std::string, ros::Publisher> pubs;
};





}
