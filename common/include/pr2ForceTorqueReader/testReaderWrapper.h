# ifndef TESTREADERWRAPPER_H
# define TESTREADERWRAPPER_H

# include <pr2_controller_interface/controller.h>
# include <realtime_tools/realtime_publisher.h>
# include <ros/ros.h>
# include <boost/shared_ptr.hpp>
# include <geometry_msgs/WrenchStamped.h>
# include <pr2_mechanism_model/robot.h>

#include <pr2ForceTorqueReader/pr2ForceTorqueReader.h>

namespace testReader{
  class testReaderWrapper: public pr2_controller_interface::Controller{

  private: 
    
    pr2ForceTorqueReader *ftReader;

    // add a low pass filter for the force torque reader
    void lowpassFT(std::vector<double> &ft); 
    std::vector<double> lastFt_;
    double torqueTimeConstant_, forceTimeConstant_;

    // publish the realtime stuff
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > realtimeLeftFTPublisher;
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > realtimeRightFTPublisher;

  public:
    virtual bool init(pr2_mechanism_model::RobotState *robot,
		      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();

  };// End of testReaderWrapper class 

}

# endif
