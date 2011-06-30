#ifndef _COMMS_SIMULATOR_H
#define _COMMS_SIMULATOR_H

#include <map>
#include <vector>
#include <string>
#include "comms_types.h"

using namespace std;

// Param Types
typedef double DOUBLE;
typedef int INT;
typedef std::vector<double> DOUBLE_VECTOR;
typedef std::vector<int> INT_VECTOR;

class CommsSimulatorBase
{
public:
	CommsSimulatorBase(const map<int, agent_state_t> &current_state);
	virtual ~CommsSimulatorBase();

	/**
	 * Run a test to determine if a packet will successfully be transmitted
	 * from source to destination.
	 *
	 * @returns 0: unknown, -1: no transmission, 1: good transmission
	 */
	virtual int TestSendPacket(int source, int destination, 
				   comms_request_reply_t *ret=NULL) { return -1; }

	virtual int TestSendPacket(int source, int destination, 
				   const vector<double> &segments,
				   comms_request_reply_t *ret=NULL) { return -1; }

	/**
	 * Get model parameter names
	 * 
	 */
	map<string, string> *GetParamTypes() { return &param_types; } 	
	void SetParam(const string &key, const DOUBLE &value);
	void SetParam(const string &key, const INT &value);
	void SetParam(const string &key, const DOUBLE_VECTOR &value);
	void SetParam(const string &key, const INT_VECTOR &value);
protected:
	const map<int, agent_state_t> &system_state;

	// map<param_name, param_type> 
	// type: DOUBLE, INT, DOUBLE_VECTOR, INT_VECTOR
	map<string, string> param_types;
	map<string, void*> param_ptrs;

	void AddParam(const string &key, const string &type, void *param_ptr);

};

// Must define this function in derived classes
CommsSimulatorBase *CreateCommsSimulator(map<int, agent_state_t> &system_state);

void LabelMaterials(link_material_properties_t &prop);

#endif
