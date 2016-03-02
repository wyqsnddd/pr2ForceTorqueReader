# include "pr2ForceTorqueReader/testReaderWrapper.h"
# include <pluginlib/class_list_macros.h>

using namespace testReader;

PLUGINLIB_EXPORT_CLASS( testReader::testReaderWrapper, pr2_controller_interface::Controller)

bool testReaderWrapper::init(pr2_mechanism_model::RobotState *robot,
			     ros::NodeHandle &n){

  ftReader = new pr2ForceTorqueReader(robot, n);

  if(!ftReader){
    std::cout<<"Failed to initiailize the force torque reader"<<std::endl;
    return false;
  }

  realtimeLeftFTPublisher.reset( new realtime_tools::RealtimePublisher<
				   geometry_msgs::WrenchStamped
				   >(n, "/selfReadLeftFT", 1000));
  realtimeRightFTPublisher.reset( new realtime_tools::RealtimePublisher<
				    geometry_msgs::WrenchStamped
				    >(n, "/selfReadRightFT", 1000));

  lastFt_.resize(6);
  lastFt_.assign(6, 0);

  forceTimeConstant_ = 100;
  torqueTimeConstant_ = 100;
  return true;
}

void  testReaderWrapper::lowpassFT(std::vector<double> &ft){ 

    for(int i = 0; i<3; i++){
      ft[i] = lastFt_[i] + 1/forceTimeConstant_*(ft[i] - lastFt_[i]);
      lastFt_[i] = ft[i];
    }
    for(int i = 3; i<6; i++){
      ft[i] = lastFt_[i] + 1/torqueTimeConstant_*(ft[i] - lastFt_[i]);
      lastFt_[i] = ft[i];
    }
}

void  testReaderWrapper::starting(){
}

void  testReaderWrapper::update(){

  std::vector<double> ftLeft, ftRight;
  ftLeft.resize(6);
  ftLeft.assign(6, 0);

  ftRight.resize(6);
  ftRight.assign(6, 0);

  ftReader->readFT(ftLeft, ftRight);
  
  lowpassFT(ftLeft);
  
  if(realtimeLeftFTPublisher->trylock()){
    realtimeLeftFTPublisher->msg_.header.stamp = ros::Time::now();
    realtimeLeftFTPublisher->msg_.wrench.force.x = ftLeft[0];
    realtimeLeftFTPublisher->msg_.wrench.force.y = ftLeft[1];
    realtimeLeftFTPublisher->msg_.wrench.force.z = ftLeft[2];
    realtimeLeftFTPublisher->msg_.wrench.torque.x = ftLeft[3];
    realtimeLeftFTPublisher->msg_.wrench.torque.y = ftLeft[4];
    realtimeLeftFTPublisher->msg_.wrench.torque.z = ftLeft[5];
    realtimeLeftFTPublisher->unlockAndPublish();
  }
  if(realtimeRightFTPublisher->trylock()){
    realtimeRightFTPublisher->msg_.header.stamp = ros::Time::now();
    realtimeRightFTPublisher->msg_.wrench.force.x = ftRight[0];
    realtimeRightFTPublisher->msg_.wrench.force.y = ftRight[1];
    realtimeRightFTPublisher->msg_.wrench.force.z = ftRight[2];
    realtimeRightFTPublisher->msg_.wrench.torque.x = ftRight[3];
    realtimeRightFTPublisher->msg_.wrench.torque.y = ftRight[4];
    realtimeRightFTPublisher->msg_.wrench.torque.z = ftRight[5];
    realtimeRightFTPublisher->unlockAndPublish();
  }

}


void  testReaderWrapper::stopping(){

  free(ftReader);
  
}
