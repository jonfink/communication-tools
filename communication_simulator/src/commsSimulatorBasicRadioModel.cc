#include "commsSimulatorBasicRadioModel.h"

#include <string>
#include <math.h>

#include <ros/ros.h>

#include <time.h>
#include <cstdlib>

#include <ctime>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

inline double dbmToPow(double x)
{
  return 0.001*pow(10,x/10.);
}

inline double PowerToBER(double P, double N)
{
  return erfc(sqrt(P/N));
}

bool PacketSuccess(double ber, int size)
{
  //double packet_drop_prob = pow(ber, (double)size);
  double packet_drop_prob = ber;
  double rand_draw = (rand() % 1000)/1000.0;

  //printf("packet_drop_prob: %f, random_draw: %f\n", packet_drop_prob, rand_draw);

  return rand_draw > packet_drop_prob;
}

CommsSimulatorBasicRadioModel::
CommsSimulatorBasicRadioModel(const map<int, agent_state_t> &current_state)
  : CommsSimulatorBase(current_state)
{
  this->AddParam(std::string("tx_power"), std::string("DOUBLE"), (void*)(&tx_power));
  tx_power = 0;

  this->AddParam(std::string("L0"), std::string("DOUBLE"), (void*)(&L0));
  L0 = 40;

  this->AddParam(std::string("fading_exp"), std::string("DOUBLE"), (void*)(&fading_exp));
  fading_exp = 2.0;

  this->AddParam(std::string("wall_PL"), std::string("DOUBLE"), (void*)(&wall_PL));
  wall_PL = 7;

  this->AddParam(std::string("rx_sensitivity"), std::string("DOUBLE"), (void*)(&rx_sensitivity));
  rx_sensitivity = -100;

  this->AddParam(std::string("noise_power"), std::string("DOUBLE"), (void*)(&noise_power));
  noise_power = -16;

  this->AddParam(std::string("fading_variance"), std::string("DOUBLE"), (void*)(&fading_variance));
  fading_variance = 25.0;

  this->AddParam(std::string("use_bearing"), std::string("INT"), (void*)(&use_bearing));
  use_bearing = 0;

  srand ( time(NULL) );

  r = gsl_rng_alloc (gsl_rng_taus);
  gsl_rng_set(r, time(0));

  link_prop.segment_depths = (double*)malloc(sizeof(double)*10);
  link_prop.segment_materials = (material_t*)malloc(sizeof(double)*10);
}

CommsSimulatorBasicRadioModel::
~CommsSimulatorBasicRadioModel()
{
  gsl_rng_free(r);
  free(link_prop.segment_depths);
  free(link_prop.segment_materials);
}

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

double CommsSimulatorBasicRadioModel::
BearingDependentFading(double th)
{
  double lambda0 = 1;
  double k0 = 2*M_PI/lambda0;
  double i0 = 1;
  double h = lambda0/4;

  double pow = 0;
  double powdb;

  if(th == -M_PI || th == -M_PI/2 || th == 0 || th == M_PI/2 || th == M_PI)
    pow = 0;
  else {
    pow = i0*(cos(h*k0) - cos(h*k0*cos(th))*(1/sin(th/2.0))*(1/cos(th/2.0)))/k0;
  }

  if(fabs(pow) < 1/10e9)
    powdb = -100;
  else
    powdb = 20*log10(fabs(pow));
 
  // computed for lamda0=1, k0 = 2*M_PI/lamda0, i0 = 1, h = lambda0/4
  double afdbmax = -11.1223;

  double max_fading = 15;

  return -max_fading + MAX(powdb-afdbmax + max_fading, 0);
}

int CommsSimulatorBasicRadioModel::
TestSendPacket(int source, int destination, 
               const vector<double> &segments,
               comms_request_reply_t *ret) 
{
  std::map<int, agent_state_t>::const_iterator src, dest;

  src = system_state.find(source);
  dest = system_state.find(destination);

  if(src == system_state.end())
    ROS_WARN("Could not find state for source address: %d", source);

  if(dest == system_state.end())
    ROS_WARN("Could not find state for destination address: %d", destination);

  bool first_wall_attenuation = false;

  double signal_power = tx_power;
  //printf("test send\n");
  //printf("\tsignal_power: %2.2f\n", signal_power);

    double dist=0;
    //printf("\t\tdist: %2.2f\n", dist);
    for(unsigned int i=0; i < segments.size(); ++i) {
      if(i % 2 == 0) {
	// AIR
        dist += segments[i];
	//printf("\t\tdist: %2.2f\n", dist);
      }
      else {
	// WALL
	dist += segments[i];
	if(!first_wall_attenuation) {
	  signal_power -= wall_PL;/*link_prop.segment_depths[i];*/
	  first_wall_attenuation = true;
	  //printf("\t\tdist: %2.2f\n", dist);
	  //printf("\tsignal_power: %2.2f (- %2.2f)\n", signal_power, wall_PL);
	}
      }
    }

    ROS_DEBUG("Total distance: %2.2f", dist);

    dist = sqrt( (src->second.x-dest->second.x)*(src->second.x-dest->second.x) +
		 (src->second.y-dest->second.y)*(src->second.y-dest->second.y) );

    ROS_DEBUG("Total distance: %2.2f (%2.2f, %2.2f to %2.2f, %2.2f)", dist, 
	      src->second.x, src->second.y, 
	      dest->second.x, dest->second.y);
    
    signal_power -= L0;
    signal_power -= 10*fading_exp*log10(dist);// + 20*log10(tx_freq) - 147.56;

    //printf("\tsignal_power: %2.2f\n", signal_power);

    if(use_bearing > 0) {
      double ang = atan2(src->second.y - dest->second.y, 
			 src->second.x - dest->second.x) - dest->second.th;
      double bearing_fading = BearingDependentFading(ang);
      signal_power += bearing_fading;
    }
    
    // Apply fading to signal_power
    if(fading_variance > 1e-3) {
      double signal_fading = gsl_ran_gaussian(r, sqrt(fading_variance));
      signal_power += signal_fading;
    }
    
    if(ret) {
      ret->txPow = tx_power;
      ret->rssi = signal_power;
    }
    
    if(PacketSuccess(PowerToBER(dbmToPow(signal_power), dbmToPow(noise_power)), 800)) {
      if(ret) 
	ret->success = 1;
      return 1;      
    }
    
    if(ret) 
      ret->success = -1;
    
    return -1;
    
}

int CommsSimulatorBasicRadioModel::
TestSendPacket(int source, int destination, 
               comms_request_reply_t *ret) 
{
  std::map<int, agent_state_t>::const_iterator src, dest;

  src = system_state.find(source);
  dest = system_state.find(destination);

  double signal_power = tx_power;

  // Do free space loss
  double dist = computeAgentStateDist(src->second, dest->second);
  signal_power -= L0 + 10*fading_exp*log10(dist);//+ 20*log10(tx_freq) - 147.56;

  if(use_bearing > 0) {
    double ang = atan2(src->second.y - dest->second.y, 
		       src->second.x - dest->second.x) - dest->second.th;
    double bearing_fading = BearingDependentFading(ang);
    /*
    printf("[BearingFading] Recveiver Heading: %2.2f, SRC Bearing (global): %2.2f, Relative Bearing: %2.2f\n", 
	   dest->second.th, 
	   atan2(src->second.y - dest->second.y, src->second.x - dest->second.x), 
	   ang);
    */
    signal_power += bearing_fading;
  }
  
  // Apply fading to signal_power
  double signal_fading = gsl_ran_gaussian(r, sqrt(fading_variance));
  signal_power += signal_fading;

  if(ret) {
    ret->txPow = tx_power;
    ret->rssi = signal_power;
  }

  if(PacketSuccess(PowerToBER(dbmToPow(signal_power), dbmToPow(noise_power)), 800)) {
    if(ret) 
      ret->success = 1;
    return 1;      
  }

  if(ret) 
    ret->success = -1;

  return -1;
}

CommsSimulatorBase *CreateCommsSimulator(map<int, agent_state_t> &system_state)
{
  return reinterpret_cast<CommsSimulatorBase*>(new CommsSimulatorBasicRadioModel(system_state));
}
