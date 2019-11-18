#include "alignment_checker/scan.h"
namespace alignment_checker{

ScanType::ScanType(): cloud_ (new pcl::PointCloud<pcl::PointXYZ>){

}
ScanType::ScanType(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input){
  cloud_ = input;
  kdtree_.setInputCloud(cloud_);
}
void ScanType::SetInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input){
  cloud_ = input;
  kdtree_.setInputCloud(cloud_);

}
void ScanType::GetOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_target, double r){
  std::vector<int> idx_input;
  std::vector<int> idx_this;
  GetNeighboors(input, r, idx_input, idx_this );
  ScanType::ExtractIndecies(input, idx_input, overlap_input);
  ScanType::ExtractIndecies(cloud_, idx_this, overlap_target);
  cout<<"overlap_input size: "<<overlap_input->size()<<endl;
  cout<<"overlap_target size: "<<overlap_target->size()<<endl;

}

void ScanType::GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double r, std::vector<int> &idx_input, std::vector<int> &idx_this ){
  std::vector<std::vector<int> > thisIdxAgg;
  std::vector<int> inputIdxAgg;
  if(input->size()==0)
    return;

  inputIdxAgg.resize(input->size(),-1);
  thisIdxAgg.resize(cloud_->size());

#pragma omp parallel num_threads(4)
  {
#pragma omp for
    for(int i=0;i<input->size();i++){
      pcl::PointXYZ p = input->points[i];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointdistRadiusSearch;
      if ( kdtree_.radiusSearch (p, r, pointIdxRadiusSearch,pointdistRadiusSearch) > 0 ){
        thisIdxAgg[i]=pointIdxRadiusSearch;
        inputIdxAgg[i] = i;
      }
    }
  }
  for(int i=0;i<input->size();i++){
    if(inputIdxAgg[i]!=-1)
      idx_input.push_back(inputIdxAgg[i]);
    for(auto &idx: thisIdxAgg[i]){
      idx_this.push_back(idx);
    }

  }
  std::cout<<"size: "<<idx_this.size()<<std::endl;
  std:sort( idx_this.begin(), idx_this.end() );

  idx_this.erase( unique( idx_this.begin(), idx_this.end() ), idx_this.end() );
  std::cout<<"after filter: "<<idx_this.size()<<std::endl;

}

double ScanType::GetInformation(const double r){


  std::vector<float> pointRadiusSquaredDistance;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<double> entropy(cloud_->size(),0);
  if(cloud_->size()==0)
    return 0;

//#pragma omp parallel num_threads(4)
  {
//#pragma omp for
    for(int i=0;i<cloud_->size();i++){
      pcl::PointXYZ p = cloud_->points[i];
      if ( kdtree_.radiusSearch (p, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        pcl::PointXYZ pc;
        //pcl::computeCentroid(*cloud_, pointIdxRadiusSearch, pc);
        Eigen::Matrix3d C;
        pcl::computeCovarianceMatrix(*cloud_,pointIdxRadiusSearch,C);

        double det = C.determinant();

        double hq = 1/2.0*log(2*M_PI*exp(det));
        entropy[i] = hq;
      }
    }
  }
  double sum = 0;
  for(int i=0;i<entropy.size();i++)
    sum+=entropy[i];
  sum/=entropy.size();
  return sum;
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




}
