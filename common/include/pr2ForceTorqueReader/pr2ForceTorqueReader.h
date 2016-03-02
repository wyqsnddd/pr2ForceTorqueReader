# ifndef PR2FORCETORQUEREADER_H
# define PR2FORCETORQUEREADER_H

# include <pr2_hardware_interface/hardware_interface.h>
# include <pr2_mechanism_model/robot.h>
/* # include <realtime_tools/realtime_publisher.h> */
# include <ros/ros.h>
# include <boost/shared_ptr.hpp>
/* # include <geometry_msgs/WrenchStamped.h> */


// coe grabbed from: yaml: robot.launch->ft.launch->wg035... yaml 
// code grabbed from: pr2_ethercat_drivers:  wg06.cpp
// code rawToCalibrate: WG06::convertFTDataSampleToWrench

class pr2ForceTorqueReader{

 private: 
  // test for the analog_in signal
  std::string analogLeftFT_, analogRightFT_;
  pr2_hardware_interface::AnalogIn *leftAnalogIn_, *rightAnalogIn_;
  pr2_hardware_interface::ForceTorque  *force_torque_left_;
  pr2_hardware_interface::ForceTorque  *force_torque_right_;
  /* std::map<std::string, AnalogIn*>::iterator it; */
    
  
  void rawToCalibrated(const pr2_hardware_interface::AnalogInState &state, 
		       const bool indicator,
		       std::vector<double> &ft);

  double calibration_coeff_right_[36];

  double calibration_coeff_left_[36];

  const double &calibration_coeff_left(unsigned row, unsigned col) const {
    //  return *(calibration_coeff_left_ + (row*6 + col) );
    return calibration_coeff_left_[row*6 + col];
  }
  double &calibration_coeff_left(unsigned row, unsigned col) {
    /* return calibration_coeff_left_[row*6 + col]; */
    return calibration_coeff_left_[row*6 + col];
  }
  const double &calibration_coeff_right(unsigned row, unsigned col) const {
    return calibration_coeff_right_[row*6 + col];
  }
  double &calibration_coeff_right(unsigned row, unsigned col) {
    return calibration_coeff_right_[row*6 + col];
  }
  double offsets_[6];
  const double &offset(unsigned ch_num) const {return offsets_[ch_num];}
  double &offset(unsigned ch_num) {return offsets_[ch_num];}

  double gains_[6];
  const double &gain(unsigned ch_num) const {return gains_[ch_num];}
  double &gain(unsigned ch_num) {return gains_[ch_num];}

  bool sensorIni(const pr2_mechanism_model::RobotState *robot);

 public:
  pr2ForceTorqueReader(const pr2_mechanism_model::RobotState *robot,
		    ros::NodeHandle &n);

  ~pr2ForceTorqueReader();

  void readFT(std::vector<double> &ftLeft, 
	      std::vector<double> &ftRight);
  void readFT(std::vector<double> &ft,
	      int indicator); // 0: left 1: right

  //  void publishFT();
  

}; // end of my FTReader

# endif
