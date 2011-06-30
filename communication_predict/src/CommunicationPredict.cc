#include "CommunicationPredict.h"

#include "comms_types.h"
#include <commsSimulatorBase.h>

CommunicationPredict::CommunicationPredict(bool use_map)
{
  if(!node.getParam("tx_power", tx_power))
    if(!node.getParam("/tx_power", tx_power))
      node.param("tx_power", tx_power, 0.0);

  if(!node.getParam("L0", L0))
    if(!node.getParam("/L0", L0))
      node.param("L0", L0, 40.0);

  if(!node.getParam("fading_exp", fading_exp))
    if(!node.getParam("/fading_exp", fading_exp))
      node.param("fading_exp", fading_exp, 3.5);

  if(!node.getParam("wall_PL", wall_PL))
    if(!node.getParam("/wall_PL", wall_PL))
      node.param("wall_PL", wall_PL, 7.0);

  if(!node.getParam("rx_sensitivity", rx_sensitivity))
    if(!node.getParam("/rx_sensitivity", rx_sensitivity))
      node.param("rx_sensitivity", rx_sensitivity, -90.0);

  if(!node.getParam("noise_power", noise_power))
    if(!node.getParam("/noise_power", noise_power))
      node.param("noise_power", noise_power, -80.0);

  if(!node.getParam("fading_variance", fading_variance))
    if(!node.getParam("/fading_variance", fading_variance))
      node.param("fading_variance", fading_variance, 25.0);

  if(!node.getParam("cutoff_rate", cutoff_rate))
    if(!node.getParam("/cutoff_rate", cutoff_rate))
      node.param("cutoff_rate", cutoff_rate, -1.0);

  printf("\n\n******************************\n\n");
  printf("noise_power: %2.2f, tx_power: %2.2f\n", noise_power, tx_power);
  printf("\n\n******************************\n\n");

  path_sim_depth = 0.5;

  this->map_set = false;

  this->use_map = use_map;

  if(use_map)
    map_sub = node.subscribe("/map_rf", 1, &CommunicationPredict::handle_map, this);

  this->sim = CreateCommsSimulator(node_state);

  sim->SetParam(string("tx_power"), tx_power);
  sim->SetParam(string("L0"), L0);
  sim->SetParam(string("fading_exp"), fading_exp);
  sim->SetParam(string("wall_PL"), wall_PL);
  sim->SetParam(string("rx_sensitivity"), rx_sensitivity);
  sim->SetParam(string("noise_power"), -500.0);
  sim->SetParam(string("fading_variance"), 0.0);
}

CommunicationPredict::
CommunicationPredict(double tx_power, double L0, double fading_exp,
		     double wall_PL, double rx_sensitivity, double noise_power,
		     double fading_variance)
{
  this->tx_power = tx_power;
  this->L0 = L0;
  this->fading_exp = fading_exp;
  this->wall_PL = wall_PL;
  this->rx_sensitivity = rx_sensitivity;
  this->noise_power = noise_power;
  this->fading_variance = fading_variance;

  this->sim = CreateCommsSimulator(node_state);

  sim->SetParam(string("tx_power"), tx_power);
  sim->SetParam(string("L0"), L0);
  sim->SetParam(string("fading_exp"), fading_exp);
  sim->SetParam(string("wall_PL"), wall_PL);
  sim->SetParam(string("rx_sensitivity"), rx_sensitivity);
  sim->SetParam(string("noise_power"), -500.0);
  sim->SetParam(string("fading_variance"), 0.0);
}

void CommunicationPredict::handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  if(this->use_map && !this->map_set) {
    ROS_INFO("Loading map with resolution %f ...\n", msg->info.resolution);
    path_sim.LoadOccupancyGrid(*msg, path_sim_depth);
    ROS_INFO("done loading map\n");
  }

  this->map_set = true;
}


double dbmToPow(double x)
{
  return 0.001*pow(10,x/10.);
}

double PowerToBER(double P, double N)
{
  return erfc(sqrt(P/N));
}

void CommunicationPredict::Predict(double xi, double yi,
				   double xj, double yj,
				   double &r, double &sigma)
{
  comms_request_reply_t ret;
  int retval;

  double fb_dist = 0.2;
  double vec_x = xj - xi;
  double vec_y = yj - yi;
  double dist = sqrt(vec_x*vec_x + vec_y*vec_y);
  vec_x /= dist;
  vec_y /= dist;

  node_state[0].x = xi - vec_x*fb_dist;
  node_state[0].y = yi - vec_y*fb_dist;

  node_state[1].x = xj + vec_x*fb_dist;
  node_state[1].y = yj + vec_y*fb_dist;

  if(map_set) {
    vector<double> segments;
    ROS_DEBUG("Getting signal path from: %2.2f, %2.2f to %2.2f, %2.2f",
	      node_state[0].x, node_state[0].y,
	      node_state[1].x, node_state[1].y);
    if(wall_PL > 1e-5) {
      double t0, tf;
      path_sim.GetSignalPath(node_state[0].x, node_state[0].y, 0.0,
                             node_state[1].x, node_state[1].y, 0.0,
                             segments, false);
    }
    retval = sim->TestSendPacket(0, 1, segments, &ret);
  }
  else {
    retval = sim->TestSendPacket(0, 1, &ret);
  }

  // Use RSSI to predict link reliability
  this->Predict(ret.rssi, fading_variance, r, sigma);

  ROS_DEBUG("RSSI: %2.2f, r: %2.2f, sigma: %2.2f", ret.rssi, r, sigma);

  //}
}

void CommunicationPredict::Predict(double rssi, double rssi_var, double &r, double &sigma)
{
  r = 0;
  sigma = 0;

  r = 1.0 - PowerToBER(dbmToPow(rssi), dbmToPow(noise_power));

  if(cutoff_rate > 0.0 && r < cutoff_rate) {
    r = 0;
  }

  if( fabs(rssi)/fabs(noise_power) > 1.1 ) {
    sigma = (6.750586336417736*pow(2,-4 - noise_power/10. + rssi/10.)*
             pow(5,-2 - noise_power/10. + rssi/10.)*rssi_var)/
      exp(2.*pow(10,-noise_power/10. + rssi/10.)) + 0.007;
  }
  else {
    sigma = (6.750586336417736*pow(2,-4 - noise_power/10. + 1.1*noise_power/10.)*
             pow(5,-2 - noise_power/10. + 1.1*noise_power/10.)*rssi_var)/
      exp(2.*pow(10,-noise_power/10. + 1.1*noise_power/10.)) + 0.007;
  }

  if(isnan(sigma)) {
    sigma = 0.007;
  }

}
