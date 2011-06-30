#ifndef __COMMUNICATION_PREDICT_H_
#define __COMMUNICATION_PREDICT_H_

#include <commsSimulatorBase.h>

#include "nav_msgs/OccupancyGrid.h"
#include "SignalPathSimulator.h"

#include "ros/ros.h"

class CommunicationPredict
{
 public:
  CommunicationPredict(bool use_map=true);
  CommunicationPredict(double tx_power, double L0, double fading_exp,
		       double wall_PL,  double rx_sensitivity, double noise_power,
		       double fading_variance);
  void Predict(double xi, double yi, double xj, double yj, double &r, double &sigma);
  void Predict(double rssi, double rssi_var, double &r, double &sigma);

  double tx_power;
  double L0;
  double fading_exp;
  double wall_PL;
  double rx_sensitivity;
  double noise_power;
  double fading_variance;

  double cutoff_rate;

 private:
  ros::NodeHandle node;
  CommsSimulatorBase *sim;
  map<int, agent_state_t> node_state;

  ros::Subscriber map_sub;
  SignalPathSimulator path_sim;
  double path_sim_depth;
  bool map_set;
  bool use_map;
  void handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};

#endif
