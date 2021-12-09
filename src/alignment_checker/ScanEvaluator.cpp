#include "alignment_checker/ScanEvaluator.h"
namespace CorAlignment {

bool datapoint::aligned(){
  double sum = 0;
  for(auto && e : perturbation_)
    sum+=fabs(e);
  return sum >0.0001;

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
  std::ofstream ofs_meta, ofs_eval;
  ofs_meta.open(par_.output_directory+std::string("/")+par_.output_meta_file);
  ofs_meta <<"method, evaluation name, scan spacing, theta range, offset rotation steps, theta error, range error"; //, scan index,
  ofs_meta<<par_.method<<", "<<par_.eval_name<<", "<<", "<<std::to_string(par_.scan_spacing)<<", "<<std::to_string(par_.theta_range)
         <<", "<<par_.offset_rotation_steps<<", "<<std::to_string(par_.theta_error)<<std::to_string(par_.range_error)<<endl;

  ofs_eval.open(par_.output_directory+std::string("/")+par_.output_eval_file);
  int i = 0;
  for(auto&& d : datapoints_){
    if(i++==0)
      ofs_eval<<"index, score1, score2, score3, aligned, error x, error y, error theta"<<std::endl;
    else
      ofs_eval<<d.index<<", "<<d.score_[0]<<", "<<d.score_[1]<<", "<<d.score_[2]<<", "<<d.aligned()<<", "<<d.perturbation_[0]<<", "<<d.perturbation_[1]<<", "<<d.perturbation_[2]<<", "<<std::endl;
  }

}
void scanEvaluator::InputSanityCheck(){

  assert( !output_directory.empty() );
  assert( !output_meta_file.empty() );
  assert( !output_eval_file.empty() );
  assert( range_error >  0.0 );
  assert( theta_range >= -DBL_EPSILON );
  assert( theta_error >= 0.0 );
  assert( !method.empty() );
  assert( scan_spacing >= 0 );
}
scanEvaluator::scanEvaluator( dataHandler_U& reader, const parameters& par, const AlignmentQuality::parameters& alignment_par): par_(par)
{
  reader_ = std::move( reader);
  InputSanityCheck();
  CreatePerturbations();


  std::vector< std::shared_ptr<PoseScan> > prev_scans;
  int index = 0;
  for (std::shared_ptr<PoseScan> current = reader_->Next(); current!=NULL; current = reader_->Next(), index++) {
    if( prev_scans.size() == par_.scan_spacing)
    {
      for(auto && verr : vek_perturbation_){
        const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);
        auto quality = AlignmentQualityFactory::CreateQualityType(prev_scans.back(), current, alignment_par, Tperturbation);
        auto res = quality->GetResiduals();
        auto score = quality->GetResiduals();
        datapoints_.push_back(datapoint(index, res, verr, score));
      }
    }
    prev_scans.push_back(current);
    if(prev_scans.size() > par_.scan_spacing){
      prev_scans.erase(prev_scans.begin());
    }
  }

  SaveEvaluation();
}


}
