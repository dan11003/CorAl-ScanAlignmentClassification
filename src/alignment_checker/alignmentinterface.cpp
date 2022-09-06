#include "alignment_checker/alignmentinterface.h"
namespace CorAlignment {

py::scoped_interpreter PythonClassifierInterface::guard_;

PythonClassifierInterface::PythonClassifierInterface(){
	py::module sys = py::module::import("sys");
	py::print(sys.attr("version"));

	// Import sklearn.linear_model
	this->sklearn_ = py::module::import("sklearn.linear_model");
}


void PythonClassifierInterface::fit(const std::string& model){
	auto py_model = sklearn_.attr("LogisticRegression")("class_weight"_a="balanced");

	// auto tuple_y = py::make_tuple(this->y_.rows(), 1);
	// auto np_y = py::module::import("numpy").attr("ndarray")("shape"_a=tuple_y,"buffer"_a=this->y_);
	// auto tuple_y = py::make_tuple(this->X_.rows(), this->X_.cols());
	// auto np_X = py::module::import("numpy").attr("ndarray")("shape"_a=tuple_X,"buffer"_a=this->X_);

	// Creating copies, might better to use refrences instead...
	auto np_y = py::cast(this->y_);
	auto np_X = py::cast(this->X_);

	this->py_clf_ = py_model.attr("fit")(np_X, np_y.attr("ravel")());
	std::cout << "Fitted model after training data" << std::endl;
}


Eigen::MatrixXd PythonClassifierInterface::predict_proba(const Eigen::MatrixXd& X){
	auto np_X = py::cast(X);
	auto result = this->py_clf_.attr("predict_proba")(np_X);
	return result.cast<Eigen::MatrixXd>();
}


Eigen::MatrixXd PythonClassifierInterface::predict(const Eigen::MatrixXd& X, const Eigen::MatrixXd& y_pred){
	auto np_y = py::cast(y_pred);
	auto np_X = py::cast(X);
	auto result = this->py_clf_.attr("predict")(np_X, np_y.attr("ravel")());

	return result.cast<Eigen::MatrixXd>();
}


void PythonClassifierInterface::AddDataPoint(Eigen::MatrixXd X_i, Eigen::MatrixXd y_i){
	this->X_.conservativeResize(this->X_.rows()+1, X_i.cols());
	this->X_.row(this->X_.rows()-1) = X_i;

	this->y_.conservativeResize(this->y_.rows()+1, y_i.cols());
	this->y_.row(this->y_.rows()-1) = y_i;
}


void PythonClassifierInterface::LoadData(const std::string& path){
	std::ifstream file;
	file.open(path);
	std::string line;
	std::vector<double> X_values;
	std::vector<double> y_values;
	unsigned int rows = 0;
	std::getline(file, line);	// header

	while (std::getline(file, line)){
		std::stringstream line_stream(line);
		std::string value;

		std::getline(line_stream, value, ',');
		y_values.push_back(std::stod(value));
		while (std::getline(line_stream, value, ',')) 
				X_values.push_back(std::stod(value));
		++rows;
	}

	const int cols = X_values.size() / rows;
	this->X_.resize(rows, cols);
	this->y_.resize(rows, 1);
	for (int i = 0; i < rows; i++){
		this->y_(i,0) = y_values.at(i);
		for (int j = 0; j < cols; j++)
			this->X_(i,j) = X_values.at(cols*i+j);
	}

	std::cout << "Loaded training data from " << path << std::endl;
}


void PythonClassifierInterface::SaveData(const std::string& path){
	std::ofstream result_file;
	result_file.open(path, std::ofstream::out);
	result_file << "aligned,score1,score2,score3" << std::endl;

	for(int i = 0; i < this->y_.rows(); i++)
	{
		result_file << y_(i) << "," << X_(i,0) << "," << X_(i,1) << "," << X_(i,2) << std::endl;
	}
	result_file.close();

	std::cout << "Saved training data in " << path << std::endl;
}


void ScanLearningInterface::AddTrainingData(s_scan& current){
	// If no previous scan
	if (this->frame_ == 0){
		this->prev_ = current;
		this->frame_++;
		return;
	}

	double range_error = 0.5;
	std::vector<std::vector<double>> vek_perturbation = {
		{0, 0, 0}, // Aligned
		{range_error, 0, 0}, {0, range_error, 0},   //Missaligned
		{-range_error, 0, 0}, {0, -range_error, 0}  //Missaligned
	};

	CorAlignment::PoseScan::Parameters posescan_par;
	posescan_par.scan_type = CorAlignment::scan_type::kstrongStructured; posescan_par.compensate = false;

	for(auto && verr : AlignmentQualityInterface::vek_perturbation_)
	{
		AlignmentQuality::parameters quality_par;
		quality_par.method = "Coral";
		const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);

		CorAlignment::PoseScan_S scan_ref = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, prev_.cldPeaks, prev_.T, Eigen::Affine3d::Identity()));
		CorAlignment::PoseScan_S scan_src = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, current.cldPeaks, current.T, Eigen::Affine3d::Identity()));

		AlignmentQuality_S quality = AlignmentQualityFactory::CreateQualityType(scan_ref, scan_src, quality_par, Tperturbation); 

		double sum = 0;
		for(auto && e : verr)
			sum+=fabs(e);
		bool aligned = sum < 0.0001;

		Eigen::MatrixXd X = Eigen::Map<Eigen::Matrix<double, 1, 3> >(quality->GetQualityMeasure().data());
		Eigen::MatrixXd y = Eigen::Matrix<double, 1, 1>{aligned}; // y(0,0) = aligned;
		this->coral_class.AddDataPoint(X, y);
	}
	this->prev_ = current;
	this->frame_++;
}


void ScanLearningInterface::PredAlignment(scan& current, s_scan& prev, std::map<std::string,double>& quality){
	CorAlignment::PoseScan::Parameters posescan_par; posescan_par.scan_type = CorAlignment::scan_type::kstrongStructured; posescan_par.compensate = false;
	
	CorAlignment::PoseScan_S scan_curr = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, current.cldPeaks, current.T, Eigen::Affine3d::Identity()));
	CorAlignment::PoseScan_S scan_prev = CorAlignment::PoseScan_S(new CorAlignment::kstrongStructuredRadar(posescan_par, prev.cldPeaks, prev.T, Eigen::Affine3d::Identity()));
	
	AlignmentQuality::parameters quality_par; quality_par.method = "Coral";
	AlignmentQuality_S quality_type = AlignmentQualityFactory::CreateQualityType(scan_curr, scan_prev, quality_par); 
	
	Eigen::MatrixXd X = Eigen::Map<Eigen::Matrix<double, 1, 3> >(quality_type->GetQualityMeasure().data());
	Eigen::MatrixXd proba = this->coral_class.predict_proba(X);

	quality.insert(std::pair<std::string,double>("Coral", proba(0,1)));
}


void ScanLearningInterface::LoadData(const std::string& dir){
	this->coral_class.LoadData(dir + "/CorAl.txt");
	this->cfear_class.LoadData(dir + "/CFEAR.txt");

	this->coral_class.fit("LogisticRegression");
	this->cfear_class.fit("LogisticRegression");
}


void ScanLearningInterface::SaveData(const std::string& dir){
	this->coral_class.SaveData(dir + "/CorAl.txt");
	this->cfear_class.SaveData(dir + "/CFEAR.txt");
}

} 