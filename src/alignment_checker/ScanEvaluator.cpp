#include "alignment_checker/ScanEvaluator.h"
namespace CorAlignment {

bool datapoint::aligned(const std::vector<double>& perturbation){
  double sum = 0;
  for(auto && e : perturbation)
    sum+=fabs(e);
  return sum < 0.0001;

}
void scanEvaluator::CreatePerturbations()
{

  // Ground truth
  vek_perturbation_.push_back({0,0,0});

  // Induce errors
  for(int i=0 ; i<par_.offset_rotation_steps ; i++){
    const double fraction = ((double)i)/((double)par_.offset_rotation_steps);
    const double pol_error = fraction*par_.theta_range;
    const double x = par_.range_error*cos(pol_error);
    const double y = par_.range_error*sin(pol_error);
    vek_perturbation_.push_back({x, y, par_.theta_error});
  }

}
void scanEvaluator::SaveEvaluation(){
    cout<<"Save Evaluation"<<endl;
  std::ofstream ofs_meta, ofs_eval;
  ofs_meta.open(par_.output_directory+std::string("/")+par_.output_meta_file);
  ofs_meta <<"evaluation name, method, radius, scan spacing, theta range, offset rotation steps, theta error, range error, scan spacing distance"<<std::endl; //, scan index,
  ofs_meta<<par_.eval_name<<","<<quality_par_.method<<","<<std::to_string(quality_par_.radius)<<","<<std::to_string(par_.scan_spacing)<<","<<std::to_string(par_.theta_range)
         <<","<<par_.offset_rotation_steps<<","<<std::to_string(par_.theta_error)<<","<<std::to_string(par_.range_error)<<","<<std::to_string(par_.scan_spacing_distance)<<endl;

  assert(!datapoints_.empty());
  ofs_eval.open(par_.output_directory+std::string("/")+par_.output_eval_file);
  ofs_eval << datapoint::HeaderToString()<<endl;;
  for(auto&& d : datapoints_)
    ofs_eval<<d.ToString()<<std::endl;
cout<<"Saved Evaluation"<<endl;
}
void scanEvaluator::InputSanityCheck(){

  assert( !par_.output_directory.empty() );
  assert( !par_.output_meta_file.empty() );
  assert( !par_.output_eval_file.empty() );
  assert( par_.range_error >  0.0 );
  assert( par_.theta_range >= -DBL_EPSILON );
  assert( par_.theta_error >= 0.0 );
  assert( par_.scan_spacing >= 1 );
}
scanEvaluator::scanEvaluator( dataHandler_U& reader, const parameters& eval_par, const AlignmentQuality::parameters& quality_par): par_(eval_par), quality_par_(quality_par), nh_("~")
{
  pub_train_data = nh_.advertise<std_msgs::Float64MultiArray>("/coral_training",1000);
  reader_ = std::move( reader);
  InputSanityCheck();
  CreatePerturbations();


  std::vector< std::shared_ptr<PoseScan> > prev_scans;
  int index = 0;

  ros::Rate rate(1.0/(0.000000001+par_.frame_delay));

  for (std::shared_ptr<PoseScan> current = reader_->Next(); current!=nullptr && ros::ok(); current = reader_->Next()) {
    if( prev_scans.size() == par_.scan_spacing )
    {
      index++;
      ros::Time t0 = ros::Time::now();
      for(auto && verr : vek_perturbation_){

        if(par_.frame_delay >0.001)
            rate.sleep();
        //cout<<verr[0]<<", "<<verr[1]<<", "<<verr[2]<<endl;
        const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);
        //cout<<endl<<prev_scans.back()->GetAffine().matrix()<<endl;
        //cout<<endl<<current->GetAffine().matrix()<<endl;
        ros::Time t1 = ros::Time::now();
        AlignmentQuality_S quality = AlignmentQualityFactory::CreateQualityType(prev_scans.back(), current, quality_par_, Tperturbation);
        ros::Time t2 = ros::Time::now();
        //cout<<"computed score"<<endl;
        if(eval_par.visualize){
          AlignmentQualityPlot::PublishPoseScan("/src", current, current->GetAffine()*Tperturbation,"/src_link");
          AlignmentQualityPlot::PublishPoseScan("/ref", prev_scans.back(), prev_scans.back()->GetAffine(),"/ref_link");
        }
        ros::Time t3 = ros::Time::now();
        std::vector<double> res = quality->GetResiduals();
        std::vector<double> quality_measure = quality->GetQualityMeasure();
        std_msgs::Float64MultiArray training_data;
        training_data.data = {((double)datapoint::aligned(verr)), quality_measure[0], quality_measure[1], quality_measure[2]};
        //cout<<"publish: "<<training_data.data<<endl;
        pub_train_data.publish(training_data);
        datapoints_.push_back(datapoint(index, res, verr, quality_measure, prev_scans.back(), current));
        timing.Document("quality",ToMs(t2-t1));
        timing.Document("plot",ToMs(t3-t2));
      }
      ros::Time t4 = ros::Time::now();
      timing.Document("all_missalignments",ToMs(t4-t0));

    }
    prev_scans.push_back(current);
    if(prev_scans.size() > par_.scan_spacing){
      prev_scans.erase(prev_scans.begin());
    }
  }


  SaveEvaluation();
  timing.PresentStatistics();

}
}
