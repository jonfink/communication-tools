#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <math.h>

#include "comms_types.h"
#include "commsSimulatorBase.h"

#include <string>
#include <map>

using namespace std;

int main(int argc, char **argv) {

  map<int, agent_state_t> system_state;

  CommsSimulatorBase *sim = CreateCommsSimulator(system_state);

  sim->SetParam(string("tx_power"), 0.0);
  sim->SetParam(string("L0"), 40.0);
  sim->SetParam(string("fading_exp"), 3.5);
  sim->SetParam(string("wall_PL"), 7.0);
  sim->SetParam(string("rx_sensitivity"), -90.0);
  sim->SetParam(string("noise_power"), -16.0);

  system_state[1].x = 0; 	system_state[1].y = 0;
  system_state[2].x = 0.707; 	system_state[2].y = 0.707;

  comms_request_reply_t ret;
  int retval;

  retval = sim->TestSendPacket(1, 2, &ret);

  printf("Initial TestSendPacket(1, 2): %d %2.2f %2.2f\n", retval, ret.txPow, ret.rssi);
  
  system_state[1].x = 0; 	system_state[1].y = 0;
  system_state[2].x = 15.0; 	system_state[2].y = 0;

  int count = 0;
  for(int i=0; i < 100; ++i) {
    retval = sim->TestSendPacket(1, 2, &ret);
    //printf("Initial TestSendPacket(1, 2): %d %2.2f %2.2f\n", retval, ret.txPow, ret.rssi);
    if(retval > 0)
      count++;
  }

  printf("Success probability: %f\n", count/100.0);
  
  return 0;
}

