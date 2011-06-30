#include "commsSimulatorBasicRangeModel.h"

#include <string>

CommsSimulatorBasicRangeModel::
CommsSimulatorBasicRangeModel(const map<int, agent_state_t> &current_state)
  : CommsSimulatorBase(current_state)
{
  this->AddParam(std::string("max_range"), std::string("DOUBLE"), (void*)(&max_range));
  max_range = 1.0;

  this->AddParam(std::string("test_int"), std::string("INT"), (void*)(&test_int));
  this->AddParam(std::string("test_int_vec"), std::string("INT_VECTOR"), (void*)(&test_int_vec));
  this->AddParam(std::string("test_double_vec"), std::string("DOUBLE_VECTOR"), (void*)(&test_double_vec));
}

int CommsSimulatorBasicRangeModel::
TestSendPacket(int source, int destination, 
	       const vector<double> &segments,
               comms_request_reply_t *ret) 
{
  return TestSendPacket(source, destination, ret);
}

int CommsSimulatorBasicRangeModel::
TestSendPacket(int source, int destination, 
               comms_request_reply_t *ret) 
{

  if( (system_state.count(source) <= 0) ||
      (system_state.count(destination) <= 0) ) {
    return 0;
  }

  std::map<int, agent_state_t>::const_iterator src, dest;

  src = system_state.find(source);
  dest = system_state.find(destination);


  double dist = computeAgentStateDist(src->second, dest->second);
  if(ret) {
    ret->success = (dist > max_range) ? -1 : 1;
  }
  if(dist > max_range)
    return -1;
	
  return 1;
}

CommsSimulatorBase *CreateCommsSimulator(map<int, agent_state_t> &system_state)
{
  return reinterpret_cast<CommsSimulatorBase*>(new CommsSimulatorBasicRangeModel(system_state));
}
