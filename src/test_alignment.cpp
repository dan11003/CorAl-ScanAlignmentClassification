

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <time.h>
#include <fstream>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/program_options.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/filter.h"
//#include "pcl/visualization/cloud_viewer.h"
#include "pcl/point_cloud.h"
#include "alignment_checker/scancomparsion.h"
#include "pcl/common/transforms.h"


namespace po = boost::program_options;

using std::cout;
using std::endl;
using std::string;

Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v) {
  Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
  Eigen::Affine3d T;
  T.linear()=q.toRotationMatrix();
  T.translation()<<v[0], v[1], v[2];
  return T;
}
//!
//! \brief vectorToAffine3d
//! \param v x y z ex yz ez
//! \return
//!
Eigen::Affine3d vectorToAffine3d(const Eigen::VectorXd &v) {

  return Eigen::Translation<double, 3>(v(0), v(1), v(2)) *
         Eigen::AngleAxis<double>(v(3), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxis<double>(v(4), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxis<double>(v(5), Eigen::Vector3d::UnitZ());
}

void ReadPosesFromFile(const std::string &filepath, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses){


  cout<<"Opening file: "<<filepath<<endl;

  string line;
  std::ifstream myfile (filepath);
  if (myfile.is_open()){
    while ( getline (myfile,line) ){
      std::vector<double> pose_compoments;
      std::vector<std::string> tokens;
      boost::split( tokens, line, boost::is_any_of(" ") );
      for(int i=1;i<tokens.size();i++){
        pose_compoments.push_back((double)atof(tokens[i].c_str()));
      }
      Eigen::Affine3d pose = TransRotvectorToAffine3d(pose_compoments);
      poses.push_back(pose);
    }
    myfile.close();
  }
  else{
    std::cout<<"couldn't open file"<<endl;
    exit(0);
  }
}
void ReadCloudsFromFile(const std::string directory, const std::string prefix, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds){

  std::string filepath=directory+"/"+prefix;
  cout<<"Searching for point clouds at :"<<filepath<<std::endl;
  int count=1;

  while(ros::ok()){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filepath+std::to_string(count++)+".pcd", *cloud) != -1 )
      clouds.push_back(cloud);
    else
      break;
  }
}
void PublishClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, ros::Publisher &pub){
  cout<<"plot :"<<clouds.size()<<" clouds"<<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for(int i=0;i<clouds.size() &&ros::ok();i++){
    (*c)+=*clouds[i];
  }
  /*pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(c);
  sor.setLeafSize(0.1, 0.1, 0.1);
  //sor.
  sor.setMinimumPointsNumberPerVoxel(3);
  sor.filter(*filtered);
  filtered->header.frame_id="/world";
pcl_conversions::toPCL(ros::Time::now(),filtered->header.stamp);*/
  c->header.frame_id="/world";
  pcl_conversions::toPCL(ros::Time::now(),c->header.stamp);

  cout<<"plot size: "<<c->size()<<endl;
  pub.publish(*c);
}
void SegmentGround(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &filtered, double height){
  unsigned int size_clouds=0;
  unsigned int filtered_size=0;
  for(int i=0;i<clouds.size();i++){

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_tmp->header.frame_id=clouds[i]->header.frame_id;
    filtered_tmp->header.stamp=clouds[i]->header.stamp;
    for(int j=0 ; j<clouds[i]->size() ;j++){
      size_clouds++;
      if((*clouds[i])[j].z > height){
        filtered_tmp->push_back((*clouds[i])[j]);
        filtered_size++;
      }
    }
    filtered_size+=filtered_tmp->size();
    filtered.push_back(filtered_tmp);
  }

  //double ratio = size_clouds==0 ? 0 : (double)(filtered_size-size_clouds)/((double)size_clouds);
  // cout<<"Segmentation, from: "<<size_clouds<<" to "<<filtered_size<<endl;
}



void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, pcl::PointCloud<pcl::Normal>::Ptr  &normal,float voxelsize=0.1){
  //cout<<"Downsample from size "<<cloud->size();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr to_filter( new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointXYZINormal p;
  p.intensity=0;
  for(int i = 0 ; i<cloud->size() ; i++){
    p.x=(*cloud)[i].x;
    p.y=(*cloud)[i].y;
    p.z=(*cloud)[i].z;
    memcpy(p.normal,(*normal)[i].normal,4*sizeof(float));
    to_filter->push_back(p);
  }

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr  tmp_cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr  tmp_normal(new pcl::PointCloud<pcl::Normal>());
  pcl::VoxelGrid<pcl::PointXYZINormal> sor;
  sor.setInputCloud (to_filter);
  sor.setLeafSize (voxelsize, voxelsize, voxelsize);
  sor.filter (*cloud_filtered);

  pcl::PointXYZ p_tmp;
  pcl::Normal n;
  for(int i=0;i<cloud_filtered->size();i++){
    pcl::PointXYZINormal tmp =  (*cloud_filtered)[i];
    memcpy(p_tmp.data, tmp.data, 3*sizeof(float));
    memcpy(n.normal, tmp.normal, 4*sizeof(float));
    tmp_cloud_out->push_back(p_tmp);
    tmp_normal->push_back(n);
  }
  cloud=tmp_cloud_out;
  normal=tmp_normal;

  cout<<" to size "<<cloud->size()<<endl;
  //r.getIndices()
}
//



void FilterClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &normals){
  unsigned int total=0,removed=0;
  for(int i=0;i<clouds.size();i++){
    total+=clouds[i]->size();
    pcl::PointCloud<pcl::Normal>::Ptr tmp_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    tmp_cloud->header=clouds[i]->header;
    for(int j = 0 ; j<(*clouds[i]).size() ; j++){
      pcl::Normal n2j = (*normals[i])[j];
      if(n2j.normal_x==n2j.normal_x && n2j.normal_y==n2j.normal_y && n2j.normal_z==n2j.normal_z){
        tmp_normal->push_back(n2j);
        tmp_cloud->push_back( (*clouds[i])[j]);
      }
      else
        removed++;
    }
    clouds[i] = tmp_cloud;
    normals[i] = tmp_normal;
  }
  cout<<"of a total of "<<total<<" points, "<<removed<<" was removed"<<endl;
}



int main(int argc, char **argv)
{

  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses;
  string filepath, directory_clouds, output_dir;
  double ground_height, radius;
  bool segment_ground, visualize, identity;

  ros::init(argc, argv, "clustering_node");
  ros::NodeHandle param("~");
  ros::Publisher cloud_pub;
  cloud_pub =param.advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
  po::options_description desc("Allowed options");
  std::string cloud_prefix;
  desc.add_options()
      ("help", "produce help message")
      ("pose-file-path",po::value<std::string>(&filepath)->default_value(std::string("/home/daniel/.ros/maps/offarla-2012_gt=1_submap=0_sizexy=250_Z=15_intrchR=8_compR=0_res=0.4_maxSensd=130_keyF=1_d=0.3_deg=3_alpha=0_dl=0_xyzir=0_mpsu=0_mnpfg=6kmnp0_sensorpose_est.txt")),"file of path containing pose files")
      ("cloud-dir", po::value<std::string>(&directory_clouds)->default_value(std::string("/home/daniel/ros/mapping_ws/src/graph_map/graph_map/scripts")),"directory of clouds")
      ("cloud-prefix",po::value<std::string>(&cloud_prefix)->default_value(std::string("cloud_")),"prefix of cloud names")
      ("output-dir",po::value<std::string>(&output_dir)->default_value(std::string("")),"output directory")
      ("ground-max-height",po::value<double>(&ground_height)->default_value(0.25),"ground height to used for filter gorund")
      ("radius", po::value<double>(&radius)->default_value(0.5),"radius of a voxel")
      ("visualize","visualize pointclouds")
      ("identity","identity affinity")
      ("segment-ground","segment gorund at a certain hight");

  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return false;
  }
  if(filepath.size()==0){
    std::cerr<<"File path empty"<<std::endl;
    exit(0);
  }
  segment_ground = vm.count("segment-ground");
  visualize = vm.count("visualize");
  identity = vm.count("identity");
  ReadPosesFromFile(filepath, poses);

  if(poses.size()==0){
    std::cerr<<"pose vector empty"<<std::endl;
    exit(0);
  }
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds, filtered_clouds;
  ReadCloudsFromFile(directory_clouds, cloud_prefix, clouds);
  if(poses.size()!=clouds.size()){
    std::cerr<<"Input data is of wrong size: clouds="<<clouds.size()<<", poses="<<poses.size()<<endl;
    exit(0);
  }
  else{
    cout<<"Found "<<poses.size()<<" poses and point clouds"<<endl;
  }


  filtered_clouds=clouds;

  boost::shared_ptr<alignment_checker::ScanComparsion> comp = boost::shared_ptr<alignment_checker::ScanComparsion>(new alignment_checker::ScanComparsion(filtered_clouds[0],filtered_clouds[1]));
  alignment_checker::VisComparsion vis;
  pcl::transformPointCloud(*clouds[0],*clouds[1],Eigen::Affine3d::Identity());
  while(ros::ok()){
    vis.PlotClouds(comp);
    usleep(1000*300);
    cout<<"again"<<endl;
    comp->CheckAlignment();
  }
  /*for(int i=2;i<filtered_clouds.size()-1 &&ros::ok() ;i++){
    char c;
    std::cin>>c;
    std::cout<<"set next"<<std::endl;
    comp.SetNext(filtered_clouds[i]);
    vis.PlotClouds(comp);


  }*/




  cout<<"finished"<<endl;


  return 0;
}



