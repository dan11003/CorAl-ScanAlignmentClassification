#include "alignment_checker/alignmentinterface.h"
namespace CorAlignment {

py::scoped_interpreter PythonClassifierInterface::guard_;

PythonClassifierInterface::PythonClassifierInterface(){
  py::module sys = py::module::import("sys");
  py::print(sys.attr("version"));

  // Import sklearn.linear_model
  this->numpy_ = py::module::import("numpy");
}


void PythonClassifierInterface::fit(const std::string& model){
  // Error checks
  if(this->X_.rows() != this->y_.rows()){
    cout << "Number of examples does not match for features and labels!" << endl;
    return;
  }
  else if(!is_finite(X_) || !is_finite(y_)){
    cout << "Matrix not finite: " << endl;
    /*if(!is_finite(X_)){
      cout << X_ << endl;
    }else{
      cout << y_ << endl;
    }*/
  }else if(1 > this->y_.rows()){
    cout << "No examples in training data!" << endl;
    return;
  }

  // Creating copies, might better to use references instead...
  auto np_y = py::cast(this->y_);
  auto np_X = py::cast(this->X_);

  if(model == "LogisticRegression"){
    auto sklearn_ = py::module::import("sklearn.linear_model");
    this->py_clf_ = sklearn_.attr("LogisticRegression")("class_weight"_a="balanced").attr("fit")(np_X, np_y);

  }
  else if(model == "DecisionTreeClassifier"){
    auto sklearn_ = py::module::import("sklearn.tree");
    this->py_clf_ = sklearn_.attr("DecisionTreeClassifier")("class_weight"_a="balanced").attr("fit")(np_X, np_y);
  }
  else{
    cout << "Error in selected model" << endl;
    return;
  }
  this->is_fit_ = true;
  std::cout << "Fitted " << model << " model after training data" << std::endl;
}


Eigen::VectorXd PythonClassifierInterface::predict_proba(const Eigen::MatrixXd& X){
  // If model is not fitted (must run fit() before)
  if(!is_fit_){
    std::cout << "Warning! Model is not fitted yet. Return probability as zero(s)" << std::endl;
    Eigen::VectorXd y_prob(X.rows());
    y_prob.setZero();
    return y_prob;
  }
  auto np_X = py::cast(X);
  auto result = this->py_clf_.attr("predict_proba")(np_X);
  auto y_prob = numpy_.attr("delete")(result, 0, 1);
  return y_prob.cast<Eigen::VectorXd>();
}


Eigen::VectorXd PythonClassifierInterface::predict(const Eigen::MatrixXd& X){
  // If model is not fitted (must run fit() before)
  if(!is_fit_){
    std::cout << "Warning! Model is not fitted yet. Return probability as zero(s)" << std::endl;
    Eigen::VectorXd y_prob(X.rows());
    y_prob.setZero();
    return y_prob;
  }
  auto np_X = py::cast(X);
  auto y_pred = this->py_clf_.attr("predict")(np_X);
  return y_pred.cast<Eigen::VectorXd>();
}


void PythonClassifierInterface::AddDataPoint(Eigen::MatrixXd X_i, Eigen::VectorXd y_i){
  if(X_.rows() == 0){
    X_ = X_i;
  }else{
    Eigen::MatrixXd X_temp = X_;
    this->X_.conservativeResize(this->X_.rows() + X_i.rows(), X_i.cols());
    this->X_<< X_temp, X_i;
  }

  if(y_.rows() == 0){
    y_ = y_i;
  }else{
    Eigen::MatrixXd y_temp = y_;
    this->y_.conservativeResize(this->y_.rows() + y_i.rows(), 1);
    this->y_<< y_temp, y_i;
  }
}

void PythonClassifierInterface::LoadData(const std::string& path){
  std::ifstream file;
  file.open(path);
  std::string line;
  std::vector<double> X_values;
  std::vector<double> y_values;
  unsigned int rows = 0;

  // Load vectors with values from file
  while (std::getline(file, line)){
    std::stringstream line_stream(line);
    std::string value;

    std::getline(line_stream, value, ',');
    y_values.push_back(std::stod(value));
    while (std::getline(line_stream, value, ','))
      X_values.push_back(std::stod(value));
    ++rows;
  }

  // Load matrices with values from vectors
  if(rows > 0){
    const int cols = X_values.size() / rows;
    this->X_.resize(rows, cols);
    this->y_.resize(rows, 1);
    for (int i = 0; i < rows; i++){
      this->y_(i) = y_values.at(i);
      for (int j = 0; j < cols; j++)
        this->X_(i,j) = X_values.at(cols*i+j);
    }
    std::cout << "Loaded training data from " << path << std::endl;
  }
  else
    std::cout << "No training data in " << path << std::endl;
}


void PythonClassifierInterface::SaveData(const std::string& path){
  std::ofstream result_file;
  result_file.open(path, std::ofstream::out);

  if(!result_file.is_open()){
    std::cout << "Could not save training data in " << path << std::endl;
    return;
  }

  // Loop over rows and columns in saved data X_ and y_
  const int cols = X_.cols();
  for(int i = 0; i < this->y_.rows(); i++){
    result_file << y_(i);
    for (int j = 0; j < cols; j++){
      result_file << "," << X_(i,j);
    }
    result_file << std::endl;
  }
  result_file.close();

  std::cout << "Saved training data in " << path << std::endl;
}


void ScanLearningInterface::AddTrainingData(const s_scan& current){
  // If no previous scan
  if (this->frame_++ == 0){
    this->prev_ = current;
    return;
  }

  // If below minimum distance to previous scan
  const double distance_btw_scans = (current.T.translation() - prev_.T.translation()).norm();
  if (distance_btw_scans < min_dist_btw_scans_){
    return;
  }

  std::vector<std::vector<double>> vek_perturbation = {
    {0, 0, 0}, // Aligned
    {range_error_, 0, 0}, {0, range_error_, 0},   //Missaligned
    {-range_error_, 0, 0}, {0, -range_error_, 0}  //Missaligned
  };

  for(auto && verr : AlignmentQualityInterface::vek_perturbation_)
  {
    const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);

    double sum = 0;
    for(auto && e : verr)
      sum+=fabs(e);
    bool aligned = sum < 0.0001;

    Eigen::VectorXd y(1);
    y(0) = aligned;

    /* CorAl */
    Eigen::MatrixXd X_CorAl = this->getCorAlQualityMeasure(current, prev_, Tperturbation);
    //cout << X_CorAl << ", " << y(0) << endl;
    this->coral_class.AddDataPoint(X_CorAl, y);

    /* CFEAR */
    Eigen::MatrixXd X_CFEAR = this->getCFEARQualityMeasure(current, prev_, Tperturbation);
    //cout << X_CFEAR << ", " << y(0) << endl;
    this->cfear_class.AddDataPoint(X_CFEAR, y);
  }
  this->prev_ = current;
}


void ScanLearningInterface::PredAlignment(scan& current, s_scan& prev, std::map<std::string,double>& quality){
  /* CorAl */
  Eigen::MatrixXd X_CorAl = this->getCorAlQualityMeasure(current, prev);
  Eigen::VectorXd y_CorAl = this->coral_class.predict_proba(X_CorAl);
  quality["Coral"] = y_CorAl(0);

  /* CFEAR */
  Eigen::MatrixXd X_CFEAR = this->getCFEARQualityMeasure(current, prev);
  Eigen::VectorXd y_CFEAR = this->cfear_class.predict_proba(X_CFEAR);
  quality["CFEAR"] = y_CFEAR(0);
}


void ScanLearningInterface::LoadData(const std::string& dir){
  this->coral_class.LoadData(dir + "/CorAl.txt");
  this->cfear_class.LoadData(dir + "/CFEAR.txt");
}


void ScanLearningInterface::SaveData(const std::string& dir){
  this->coral_class.SaveData(dir + "/CorAl.txt");
  this->cfear_class.SaveData(dir + "/CFEAR.txt");
}

void ScanLearningInterface::FitModels(const std::string& model){
  cout << "Fit CorAl model" << endl;
  this->coral_class.fit(model);
  cout << "Fit CFEAR model" << endl;
  this->cfear_class.fit(model);
}


Eigen::Matrix<double, 1, 2> ScanLearningInterface::getCorAlQualityMeasure(const s_scan& current, const s_scan& prev, const Eigen::Affine3d Toffset){
  CorAlignment::PoseScan::Parameters posescan_par; posescan_par.scan_type = CorAlignment::scan_type::kstrongStructured; posescan_par.compensate = false;
  CorAlignment::PoseScan_S scan_curr = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, current.cldPeaks, current.T, Eigen::Affine3d::Identity()));
  CorAlignment::PoseScan_S scan_prev = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, prev.cldPeaks, prev.T, Eigen::Affine3d::Identity()));

  AlignmentQuality::parameters quality_par; quality_par.method = "Coral"; quality_par.radius = 1.0; quality_par.weight_res_intensity = false;

  AlignmentQuality_S quality_type = AlignmentQualityFactory::CreateQualityType(scan_curr, scan_prev, quality_par, Toffset);

  Eigen::Matrix<double, 1, 2> quality_measure = Eigen::Map<Eigen::Matrix<double, 1, 2> >(quality_type->GetQualityMeasure().data());
  // cout<<"CorAl quality measure: \n"<<quality_measure<<endl<<endl;
  return quality_measure;
}


Eigen::Matrix<double, 1, 3> ScanLearningInterface::getCFEARQualityMeasure(const s_scan& current, const s_scan& prev, const Eigen::Affine3d Toffset){
  CorAlignment::PoseScan::Parameters posescan_par; posescan_par.scan_type = CorAlignment::scan_type::cfear; posescan_par.compensate = false;
  CorAlignment::PoseScan_S scan_curr = CorAlignment::PoseScan_S(new CorAlignment::CFEARFeatures(posescan_par, current.CFEAR, current.T, Eigen::Affine3d::Identity()));
  CorAlignment::PoseScan_S scan_prev = CorAlignment::PoseScan_S(new CorAlignment::CFEARFeatures(posescan_par, prev.CFEAR, prev.T, Eigen::Affine3d::Identity()));

  AlignmentQuality::parameters quality_par; quality_par.method = "P2L";
  // quality_par.weight_res_intensity = true
  AlignmentQuality_S quality_type = AlignmentQualityFactory::CreateQualityType(scan_curr, scan_prev, quality_par, Toffset);

  Eigen::Matrix<double, 1, 3> quality_measure = Eigen::Map<Eigen::Matrix<double, 1, 3> >(quality_type->GetQualityMeasure().data());
  // cout<<"CFEAR quality measure: \n"<<quality_measure<<endl<<endl;
  return quality_measure;
}

}
