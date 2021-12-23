#include "alignment_checker/Utils.h"

namespace alignment_checker {

void SetScanLocations(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds,  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses){
  for(int i = 0 ; i<clouds.size() ; i++)
    SetScanLocation(clouds[i], poses[i]);
}
void SetScanLocation(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud,  Eigen::Affine3d &pose){
  cloud->sensor_origin_ = Eigen::Vector4f(pose.translation()(0), pose.translation()(1), pose.translation()(2), 1);
}

void FilterCloudsByDistance(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses, double radius){
  float squaredradius = radius*radius;
  for(int i=0;i<clouds.size();i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p_sensor;
    p_sensor.x = poses[i].translation()(0);
    p_sensor.y = poses[i].translation()(1);
    p_sensor.z = poses[i].translation()(2);

    for(auto j : clouds[i]->points){
      if((j.x-p_sensor.x)*(j.x-p_sensor.x) + (j.y-p_sensor.y)*(j.y-p_sensor.y) + (j.z-p_sensor.z)*(j.z-p_sensor.z) > squaredradius)
        tmp->push_back(j);
    }
    clouds[i] = tmp;
  }
}

void ReadPosesFromFile(const std::string &filepath, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses){


  cout<<"Opening file: "<<filepath<<endl;

  string line;
  int index =0;
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
void ReadCloudsFromFile(const std::string directory, const std::string prefix, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, int start_index){

  std::string filepath=directory+"/"+prefix;
  int count = start_index;
  cout<<"Searching for point clouds at :"<<filepath+std::to_string(count)+".pcd"<<std::endl;

  while(ros::ok()){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filepath+std::to_string(count++)+".pcd", *cloud) != -1 )
      clouds.push_back(cloud);
    else
      break;
  }
}

Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v) {
  Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
  Eigen::Affine3d T;
  T.linear()=q.toRotationMatrix();
  T.translation()<<v[0], v[1], v[2];
  return T;
}

Eigen::Affine3d VectorToAffine3dxyez(const std::vector<double>& vek){
  assert(vek.size()==3);
  return VectorToAffine3dxyez(vek[0], vek[1], vek[2]);
}
Eigen::Affine3d VectorToAffine3d(const Eigen::Matrix<double, 6,1> &v) {

  return Eigen::Translation<double, 3>(v(0), v(1), v(2)) *
      Eigen::AngleAxis<double>(v(3), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(v(4), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(v(5), Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d VectorToAffine3dxyez(double x, double y, double theta) {

  return Eigen::Translation<double, 3>(x,y,0) *
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(theta, Eigen::Vector3d::UnitZ());
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



void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, pcl::PointCloud<pcl::Normal>::Ptr  &normal,float voxelsize){
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

void PublishCloud(const std::string& topic, pcl::PointCloud<pcl::PointXYZ>& cld){
  static std::map<std::string,ros::Publisher> pubs;
  std::map<std::string, ros::Publisher>::iterator it = pubs.find(topic);
  if (it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] =  nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic,100);
    it = pubs.find(topic);
  }
  //cout<<"publish to "<<topic<<endl;

  it->second.publish(cld);
}


pcl::PointCloud<pcl::PointXY>::Ptr pcl3dto2d(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, std::vector<double>& intensity){
  pcl::PointCloud<pcl::PointXY>::Ptr output = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());
  output->resize(input->size());
  intensity.resize(input->size());
  assert(input !=NULL);
  int index = 0;
  for (auto && p : input->points) {
    output->points[index].x = p.x;
    output->points[index].y = p.y;
    intensity[index++] = p.intensity;
  }
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr pclAddIntensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const std::vector<double>& intensity){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cld(new pcl::PointCloud<pcl::PointXYZI>());
  assert( intensity.size() == input->size() );
  cld->resize(input->size());
  for(int i=0;i<input->size();i++){
    cld->points[i].x = input->points[i].x;
    cld->points[i].y = input->points[i].y;
    cld->points[i].z = input->points[i].z;
    cld->points[i].intensity = intensity[i];
  }
  return cld;
}



}
