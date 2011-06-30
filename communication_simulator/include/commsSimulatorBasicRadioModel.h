#ifndef _COMMS_SIMULATOR_BASIC_RANGE_H_
#define _COMMS_SIMULATOR_BASIC_RANGE_H_

#include<vector>

#include "commsSimulatorBase.h"
#include <gsl/gsl_rng.h>

class CommsSimulatorBasicRadioModel : protected CommsSimulatorBase
{
 public:
  CommsSimulatorBasicRadioModel(const map<int, agent_state_t> &current_state);
  virtual ~CommsSimulatorBasicRadioModel();

  virtual int TestSendPacket(int source, int destination, 
                             comms_request_reply_t *ret=NULL);
  virtual int TestSendPacket(int source, int destination, 
			     const vector<double> &segments, 
                             comms_request_reply_t *ret=NULL);
 private:
  double tx_power;

  double L0;
  double fading_exp;
  double wall_PL;

  double rx_sensitivity;

  double noise_power;

  double fading_variance;

  int use_bearing;

  link_material_properties_t link_prop;

  gsl_rng *r;  // gsl random number generator

  double BearingDependentFading(double th);
};

#endif
