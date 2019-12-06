#include "alignment_checker/alignmenttester.h"

namespace alignment_checker {

AlignmentTester::AlignmentTester(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds,std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses)
{
  clouds_ = clouds;
  poses_ = poses;
  srand((unsigned)time(NULL));
  ros::NodeHandle nh("~");
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("points11", 10);
  pub2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("points22", 10);

}
void AlignmentTester::PerformAndSaveTest(const std::string &dir){
  for(int i=0 ; i+1<clouds_.size() && ros::ok();i++){
    boost::shared_ptr<ScanComparsion> comp = boost::shared_ptr<ScanComparsion>(new ScanComparsion(cloud_small_terror_[i+1],clouds_[i]));
    comp->StoreComparsionData(dir,i);
  }
}

double AlignmentTester::GausianNoiseGen(double dev){
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0,1);

  return distribution(generator)*dev;
}
void AlignmentTester::AllocateScans(){
  cloud_small_terror_.resize(clouds_.size());
  small_terror_.resize(clouds_.size());
  for(int i=0;i<clouds_.size();i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudcpy(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_small_terror_[i] = cloudcpy;
  }
}
void AlignmentTester::ReAlignScansSmallOffset(){
  cout<<"Small error"<<endl;
  AllocateScans();
  for(int i=0;i<cloud_small_terror_.size()&& ros::ok();i++){
    Eigen::Matrix<double,6,1> offset;
    const double sigmax=0.1,sigmay=0.1,sigmaez=0.05;
    double x=GausianNoiseGen(sigmax), y=GausianNoiseGen(sigmay), ez=GausianNoiseGen(sigmaez);
    while(x<sigmax/2||y<sigmay/2||ez<sigmaez/2){
      x=GausianNoiseGen(sigmax);
      y=GausianNoiseGen(sigmay);
      ez=GausianNoiseGen(sigmaez);
    }
    offset<<x, y, 0, 0, 0, ez;

    Eigen::Affine3d Toffset = poses_[i]*VectorToAffine3d(offset)*poses_[i].inverse();
    small_terror_[i] = Toffset;
    cloud_small_terror_[i]->clear();
    pcl::transformPointCloud( *clouds_[i], *cloud_small_terror_[i], Toffset);
  }

}
void AlignmentTester::ReAlignScansNoOffset(){
  cout<<"No error"<<endl;
  AllocateScans();
  for(int i=0;i<cloud_small_terror_.size()&& ros::ok();i++){
    small_terror_[i] = poses_[i];
    cloud_small_terror_[i]->clear();
    *cloud_small_terror_[i] = *clouds_[i];
  }

}






}
