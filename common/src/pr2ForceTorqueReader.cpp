#include "pr2ForceTorqueReader/pr2ForceTorqueReader.h"

pr2ForceTorqueReader::pr2ForceTorqueReader(const pr2_mechanism_model::RobotState *robot, 
				     ros::NodeHandle &n){

  if(!sensorIni(robot)){
    std::cout<<"Failed to initialize parameters for the force torque reader."<<std::endl;
  }


}

bool pr2ForceTorqueReader::sensorIni(const pr2_mechanism_model::RobotState *robot){
  try{				  
    std::string stringPool[2];
    int count = 0;
    for (std::map<std::string, pr2_hardware_interface::AnalogIn*>::iterator it=robot->model_->hw_->analog_ins_.begin(); 
	 it!=robot->model_->hw_->analog_ins_.end(); 
	 ++it){
      std::cout<<" "<<it->first.c_str()<<" => "<<it->second->name_.c_str()<<std::endl;
      stringPool[count] = it->first.c_str();
      count++;
    }

    leftAnalogIn_ = robot->model_->hw_->getAnalogIn(stringPool[0]);
    rightAnalogIn_ = robot->model_->hw_->getAnalogIn(stringPool[1]);

    if((!leftAnalogIn_)){
      throw "can not find the specified left FT analog signal";
    }	
    if((!rightAnalogIn_)){
      throw "can not find the specified right FT analog signal";
    }

    double calibration_coeff_left_buffer[36] = 
      {-789.2529808604213, 110.50589298551317, 1611.573936747139, -22698.95430844381, -694.5408984624096, 22499.74827114577,
       -1218.6338153838442, 26641.667307091735, -702.6874849257251, -13047.925973765452, 1428.3747237360687, -13004.94546382568,
       33055.28048169259, 1187.2144952938188, 32391.85159516855, 1052.321836359097, 33553.89608431357, 1297.6526641278303,
       -1.8378663217437679, 187.95358597293105, -537.752881389227, -114.0797890420505, 539.5378654001714, -66.08194129153888,
       610.5593893046305, 24.353400861298333, -321.9852658995056, 147.6956188771915, -298.3013537154365, -171.3945850115275,
       25.276384689531938, -338.37069368221313, 9.674322134112838, -325.3942933445349, 5.998206141619876, -331.4581972133654};

    for(unsigned i = 0; i<36; i++){
      calibration_coeff_left_[i]  = calibration_coeff_left_buffer[i];
    }
    double calibration_coeff_right_buffer[36] = 
      {-283.1615971783, 73.2312005913, 1481.8368222111, -22355.0077146435, -838.684779716, 22438.2119332104,
       -2779.8342508193, 25519.7947500348, 183.5267328333, -13082.7025108904, 1397.1029593229, -12999.5796646489,
       31289.7806619195, 1545.3548829281, 32686.8715660433, 1591.9774760058, 33020.3400981701, 2780.8698159067,
       -20.5482519172, 171.4960664527, -521.2264692735, -112.510887817, 531.9756510862, -45.2042363007,
       590.7118172833, 24.0211112676, -324.0978843345, 135.3290638582, -298.026119596, -172.9977235325,
       35.9057267938, -320.3359586188, 19.2241724888, -328.2061001705, 8.7899614816, -334.3315910207};
  
    for(unsigned i = 0; i<36; i++){
      calibration_coeff_right_[i]  = calibration_coeff_right_buffer[i];
    }
    for(unsigned i = 0; i < 6; i++ ){
      offsets_[i] = 0.0;
      gains_[i] = 50.34;
    }


    std::cout<<"Initialized the force torque sensor readers"<<std::endl;

    return true; 
  }catch(const char* msg){
    std::cout<<msg<<std::endl;
    return false;
  }
}

void pr2ForceTorqueReader::rawToCalibrated(const pr2_hardware_interface::AnalogInState &state,
					const bool indicator,
					std::vector<double> &ft){
  double in[6];
  for (unsigned i=0; i<6; ++i){
    int raw_data = state.state_[i];
    in[i] = (double(raw_data) - offset(i)) / ( gain(i) * double(1<<16) );
  }

  double out[6];
  if(indicator){ // true -> left 
    for (unsigned i=0; i<6; ++i){
      double sum = 0.0;
      for (unsigned j=0; j<6; ++j){
	sum += calibration_coeff_left(i,j) * in[j];
      }
      out[i] = sum;
    }
  }else{        // false -> right 
    for (unsigned i=0; i<6; ++i){
      double sum = 0.0;
      double test = 0.0;
      for (unsigned j=0; j<6; ++j){
	// sum += calibration_coeff_right(i,j) * in[j];
	test += calibration_coeff_right(i , j) * in[j];
	sum += calibration_coeff_right(i , j) * in[j];
      }
      out[i] = sum;

    }
  }
  ft[0]  = out[0];
  ft[1]  = out[1];
  ft[2]  = out[2];
  ft[3]  = out[3];
  ft[4]  = out[4];
  ft[5]  = out[5];

}

void pr2ForceTorqueReader::readFT(std::vector<double> &ftLeft, 
			       std::vector<double> &ftRight){
  // read the measurements into two vectors
 
  if (leftAnalogIn_->state_.state_.size() != 6){
    ROS_ERROR_THROTTLE(5.0, "NetFTExampleController: AnalogInput is has unexpected size %d", 
		       int(leftAnalogIn_->state_.state_.size()));
    return;
  }
  if (rightAnalogIn_->state_.state_.size() != 6){
    ROS_ERROR_THROTTLE(5.0, "NetFTExampleController: AnalogInput is has unexpected size %d", 
		       int(rightAnalogIn_->state_.state_.size()));
    return;
  }

  // if(leftAnalogIn_ != NULL){
  // if(
  rawToCalibrated(leftAnalogIn_->state_, true, ftLeft);
  rawToCalibrated(rightAnalogIn_->state_, false, ftRight);

}
void pr2ForceTorqueReader::readFT(std::vector<double> &ft,
				  int key){

  if(key == 0){
    if (leftAnalogIn_->state_.state_.size() != 6){
      ROS_ERROR_THROTTLE(5.0, "NetFTExampleController: AnalogInput is has unexpected size %d", 
			 int(leftAnalogIn_->state_.state_.size()));
      return;
    }
    rawToCalibrated(leftAnalogIn_->state_, true, ft);
  }else if(key == 1){
    if (rightAnalogIn_->state_.state_.size() != 6){
      ROS_ERROR_THROTTLE(5.0, "NetFTExampleController: AnalogInput is has unexpected size %d", 
			 int(rightAnalogIn_->state_.state_.size()));
      return;
    }
    rawToCalibrated(rightAnalogIn_->state_, false, ft);
  }else{
    return;
  }
}
