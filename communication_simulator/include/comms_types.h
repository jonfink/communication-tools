#ifndef _COMMS_TYPES_
#define _COMMS_TYPES_

#include <math.h>

/** Right now agent state is _just_ a function of position in the x,y plane.
 *  
 *  Note: we can add other state-variables such as z-height or TX power.
 */ 
typedef struct agent_state {
  double x;                 //! X-position (m)
  double y;                 //! Y-position (m)
  double th;                //! Orientation (rad)
} agent_state_t;

enum material_t {AIR, WALL};

/** Description of the environment materials that the wireless link must pass
 * through from source to destination.
 */
typedef struct link_material_properties {
  int segments_count;
  double *segment_depths;
  material_t *segment_materials;
} link_material_properties_t;

/** Specify request to the communication simulator.  Behavior like broadcast or 
 *  multicast are handled by the CommunicationManager.
 * 
 *  Note: we can add more information to this request such as packet length.
 */
typedef struct comms_request {
  int sender_id;             //! Unique ID for the sending agent
  int receiver_id;           //! Unique ID for the receiving agent
} comms_request_t;

/** The communication channel simulator must fill in this information for each
 *  comms evaluation.
 */
typedef struct comms_request_reply {
  int sender_id;             //! Copied from request
  int receiver_id;           //! Coped from request
  int success;               //! -1: no transmission, 1: good transmission
  double rssi;                  //! Received Signal-Stregth Indication (default dBm)
  double txPow;                 //! Transmit Power (dBm)
} comms_request_reply_t;

/** Convienience function for inter-agent distance */
inline double computeAgentStateDist(const agent_state_t &a, const agent_state_t &b)
{
  return sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

#endif
