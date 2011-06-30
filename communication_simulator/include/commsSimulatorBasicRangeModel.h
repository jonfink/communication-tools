#ifndef _COMMS_SIMULATOR_BASIC_RANGE_H_
#define _COMMS_SIMULATOR_BASIC_RANGE_H_

#include<vector>

#include "commsSimulatorBase.h"

class CommsSimulatorBasicRangeModel : protected CommsSimulatorBase
{
 public:
  CommsSimulatorBasicRangeModel(const map<int, agent_state_t> &current_state);

  virtual int TestSendPacket(int source, int destination, 
                             comms_request_reply_t *ret=NULL);
  virtual int TestSendPacket(int source, int destination, 
			     const vector<double> &segments,
                             comms_request_reply_t *ret=NULL);
 private:
  double max_range;
  int test_int;

  std::vector<int> test_int_vec;
  std::vector<double> test_double_vec;
};

#endif
