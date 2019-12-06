#include "alignment_checker/scan.h"
namespace alignment_checker{

ScanType::ScanType(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, bool downsample):cloud_with_information_(new pcl::PointCloud<pcl::PointXYZI> ) {
  if(input==NULL){
    cerr<<"NULL Input"<<endl;
    exit(0);
  }
  cloud_ = input;
  kdtree_.setInputCloud(cloud_);
}

void ScanType::SetInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input){
  cloud_ = input;
  kdtree_.setInputCloud(cloud_);
  entropy_.clear();
  valid_pnt_.clear();

}
void ScanType::GetOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_target, double radius){
  std::vector<int> idx_input;
  std::vector<int> idx_this;
  GetNeighboors(input, radius, idx_input, idx_this );
  ScanType::ExtractIndecies(input, idx_input, overlap_input);
  ScanType::ExtractIndecies(cloud_, idx_this, overlap_target);
}

void ScanType::GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double radius, std::vector<int> &idx_input, std::vector<int> &idx_this ){
  std::vector<std::vector<int> > thisIdxAgg;
  std::vector<int> inputIdxAgg;
  if(input ==NULL || input->size()==0){
    cerr<<"Input error"<<endl;
    return;
  }


  inputIdxAgg.resize(input->size(),-1);
  thisIdxAgg.resize(input->size());


#pragma omp parallel num_threads(8)
  {
#pragma omp for
    for(int i=0;i<input->size();i++){
      pcl::PointXYZ p = input->points[i];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointdistRadiusSearch;
      if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch,pointdistRadiusSearch) > 3 ){
        inputIdxAgg[i] = i;
        thisIdxAgg[i]=pointIdxRadiusSearch;
      }
    }
  }


  for(int i=0;i<inputIdxAgg.size();i++){
    if(inputIdxAgg[i]!=-1)
      idx_input.push_back(inputIdxAgg[i]);
  }

  for(int i=0;i<inputIdxAgg.size();i++){
    for(int j=0;j< thisIdxAgg[i].size();j++){
      idx_this.push_back(thisIdxAgg[i][j]);
    }
  }


std:sort( idx_this.begin(), idx_this.end() );
  idx_this.erase( unique( idx_this.begin(), idx_this.end() ), idx_this.end() );


}

double ScanType::CalculateInformation(const double radius){
  if(cloud_->size()==0)
    return 0;

  if(entropy_.size()==0){
    entropy_.resize(cloud_->size(), default_entropy_);
    valid_pnt_.resize(cloud_->size(), false);
#pragma omp parallel num_threads(1)
    {
#pragma omp for
      for(int i=0;i<cloud_->size();i++){
        pcl::PointXYZ p = cloud_->points[i];
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<int> pointIdxRadiusSearch;
        if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3 ){
          pcl::PointXYZ pc;
          Eigen::Matrix3d C;
          pcl::computeCovarianceMatrix(*cloud_,pointIdxRadiusSearch,C);
          double det = C.determinant();//exp(1.0)*det
          double hq = 1/2.0*log(2*M_PI*exp(1.0)*det);

          if(!std::isnan(hq)){
            entropy_[i] = hq;
            valid_pnt_[i] = true;
          }
        }
      }
    }
  }

  double sum = 0;
  int count=0;
  for(int i=0 ; i<entropy_.size() ; i++){
    if(valid_pnt_[i]){
      sum += entropy_[i];
      count++;
    }
  }
  if(count>0)
    average_entropy_  = sum/=((double)count);
  else
    average_entropy_  = 0;
  return average_entropy_;

}

void ScanType::ExtractIndecies(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, std::vector<int> &indecies, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered){
  if(input ==NULL || input->size()<indecies.size() || filtered==NULL ){
    std::cerr<<"Error Extract indecies"<<std::endl;
    return;
  }
  filtered->resize(indecies.size());
  for(int i=0;i<indecies.size();i++){
    if(indecies[i]<input->size())
      filtered->points[i] = input->points[indecies[i]];
    else
      std::cerr<<"indecies out of range"<<std::endl;
  }


}
pcl::PointCloud<pcl::PointXYZI>::Ptr ScanType::GetScanWithInformation(){

  if(cloud_with_information_==NULL){
    cerr<<"NULL cloud"<<endl;
    exit(0);
  }
  cloud_with_information_->resize(cloud_->size());

  /*#pragma omp parallel num_threads(8)
  {
#pragma omp for*/
  for(int i=0;i<cloud_->size();i++){
    pcl::PointXYZI p;
    p.x = cloud_->points[i].x;
    p.y = cloud_->points[i].y;
    p.z = cloud_->points[i].z;
    p.intensity = entropy_[i]*10;//255*1/(1+exp(-entropy_[i]));
    cloud_with_information_->points[i] = p;
  }
  //}
  return cloud_with_information_;
}




}
