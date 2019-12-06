#include "alignment_checker/viewer.h"

namespace alignment_checker {
viewer::viewer(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> &poses)
{
  offset_<<0, 0, 0, 0, 0, 0;
  downsample_ = true;
  poses_ = poses;
  clouds_ = clouds;
  KeyboardInputThread();

}
void viewer::SetScans(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr target;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);

  if(target_idx_+1>=clouds_.size()|| target_idx_ <0 ||clouds_[target_idx_]==NULL ||clouds_[target_idx_+1]==NULL ){
    cerr<<"index out of range"<<endl;
    return;
  }

  target = clouds_[target_idx_];
  Eigen::Affine3d Toffset = poses_[target_idx_+1]*VectorToAffine3d(offset_)*poses_[target_idx_+1].inverse();
  pcl::transformPointCloud(*clouds_[target_idx_+1], *src, Toffset);

  comp_ = boostScan(new ScanComparsion(src, target, 0.5, downsample_));
  vis_.PlotClouds(comp_);
}

void viewer::KeyboardInputThread(){
  char input=' ';
  std::cout<<"Welcome to the view score tool"<<std::endl;

  do
  {
    input=' ';
    std::cin.clear();
    std::cin>>input;
    if(input=='w')     { offset_(0)+=step_; new_input_=true;}
    else if(input=='s'){ offset_(0)-=step_; new_input_=true;}
    else if(input=='a'){ offset_(1)-=step_; new_input_=true;}
    else if(input=='d'){ offset_(1)+=step_; new_input_=true;}
    else if(input=='z'){ offset_(2)-=step_; new_input_=true;}
    else if(input=='x'){ offset_(2)+=step_; new_input_=true;}
    else if(input=='q'){ offset_(5)-=step_/3.0; new_input_=true;}
    else if(input=='e'){ offset_(5)+=step_/3.0; new_input_=true;}
    else if(input=='c'){ target_idx_+=1;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='v'){ target_idx_-=1;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='b'){ target_idx_+=50;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='n'){ target_idx_-=50;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='r'){ offset_<<0,0,0,0,0,0; new_input_=true;}
    else if(input=='g'){ downsample_=!downsample_; new_input_=true;}
    if(new_input_)
      SetScans();

    new_input_ = false;
  }while(input!='t');
  cout<<"Exit"<<endl;
  exit(0);

}

}
