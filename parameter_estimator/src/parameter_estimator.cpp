//  March/2020, Auterion, Jaeyoung Lim, jaeyoung@auterion.com

#include "parameter_estimator/parameter_estimator.h"


using namespace Eigen;
using namespace std;
//Constructor
ParameterEstimator::ParameterEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &ParameterEstimator::cmdloopCallback, this); // Define timer for constant loop rate
  mavpose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &ParameterEstimator::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwist_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1, &ParameterEstimator::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());

  estimator_ = std::make_shared<Estimator>();
}
ParameterEstimator::~ParameterEstimator() {
  //Destructor
}

void ParameterEstimator::cmdloopCallback(const ros::TimerEvent& event){
  estimator_->UpdateState(mav_pos_, mav_vel_, mav_att_);
}
