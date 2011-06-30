#include <set>
#include <map>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "p2p_msgs/p2p.h"

#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

#include "comms_types.h"
#include "commsSimulatorBase.h"

#include "SignalPathSimulator.h"

#define MAX_MESSAGE_SIZE 1024

class RadioSimNode;

class RadioSim
{
public:
  // Constructor
  RadioSim();

  // Destructor
  ~RadioSim();

  void OnP2PMessage(const p2p_msgs::p2pConstPtr &_msg);

  void SendPkt(int src, int dest, p2p_msgs::p2p &_msg);
  void PublishPkt(int dest, const p2p_msgs::p2p &_msg);

  void UpdateAgentState(int address);

  double tx_power;
  double L0;
  double fading_exp;
  double wall_PL;
  double rx_sensitivity;
  double noise_power;
  double fading_variance;

private:
  ros::NodeHandle node;
  ros::Publisher p2p_pub;
  ros::Subscriber map_sub;

  tf::TransformListener tf_listener;

  map<int, agent_state_t> node_state;
  map<int, string> node_name;
  map<int, RadioSimNode*> node_objs;

  SignalPathSimulator path_sim;
  double path_sim_depth;
  bool map_set;
  void handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  CommsSimulatorBase *sim;

  std::string frame_id;
  std::string base_frame_id;
  std::string radio_prefix;
};

class RadioSimNode
{
private:
  RadioSim *parent;
  ros::NodeHandle node;
  int address;
public:
  string name;
  ros::Subscriber node_send;
  ros::Publisher node_recv;

  RadioSimNode();
  RadioSimNode(RadioSim *parent_, int address, const string &name, const string &radio_prefix=string(""));
  void OnP2PMessage(const p2p_msgs::p2pConstPtr &_msg);
  string GetFrameID();
};

////////////////////////////////////////////////////////////////////////////////
// Constructor
RadioSim::RadioSim()
{
  node.param("tx_power", tx_power, 0.0);
  node.param("L0", L0, 40.0);
  node.param("fading_exp", fading_exp, 3.5);
  node.param("wall_PL", wall_PL, 7.0);
  node.param("rx_sensitivity", rx_sensitivity, -90.0);
  node.param("noise_power", noise_power, -16.0);
  node.param("fading_variance", fading_variance, 25.0);

  ros::NodeHandle pnh("~");

  pnh.param("frameid", frame_id, string("base_link"));
  pnh.param("base_frame_id", base_frame_id, string("/map"));
  node.param("radio_prefix", radio_prefix, string(""));

  path_sim_depth = 0.5;
  this->map_set = false;
  map_sub = node.subscribe("/map_rf", 1, &RadioSim::handle_map, this);

  int use_bearing;
  pnh.param("use_bearing", use_bearing, 0);

  int num_nodes;
  node.param("num_nodes", num_nodes, 1);

  for(int i=0; i < num_nodes; ++i) {
    stringstream name_key, address_key;
    int address;
    string name;

    name_key << "name" << i;
    address_key << "address" << i;

    node.param(name_key.str(), name, name_key.str());
    node.param(address_key.str(), address, i);

    node_state.insert(make_pair(address, agent_state_t()));
    node_name.insert(make_pair(address, name));

    node_objs.insert(make_pair(address,
			       new RadioSimNode(this, address, name, radio_prefix)));
  }

  ROS_INFO("Nodes are:");
  for(map<int, RadioSimNode*>::iterator i = this->node_objs.begin();
      i != this->node_objs.end();
      ++i)
    {
      ROS_INFO("    %d (%s) ", i->first, i->second->name.c_str());
    }

  this->sim = CreateCommsSimulator(node_state);

  sim->SetParam(string("tx_power"), tx_power);
  sim->SetParam(string("L0"), L0);
  sim->SetParam(string("fading_exp"), fading_exp);
  sim->SetParam(string("wall_PL"), wall_PL);
  sim->SetParam(string("rx_sensitivity"), rx_sensitivity);
  sim->SetParam(string("noise_power"), noise_power);
  sim->SetParam(string("fading_variance"), fading_variance);
  sim->SetParam(string("use_bearing"), use_bearing);

  return;
}

void RadioSim::UpdateAgentState(int address)
{
  string tf_name = node_name[address]+string("/base");
  tf::StampedTransform transform;
  try{
    tf_listener.lookupTransform(base_frame_id, tf_name,
				ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  // Convert map_radio_origin.pose.orienation into euler angles
  btMatrix3x3 mat(transform.getRotation());
  double yaw, pitch, roll;
  mat.getEulerZYX(yaw, pitch, roll);

  // put yaw in 0, 6.28 range
  while(yaw < 0) {
    yaw += 2*M_PI;
  }

  ROS_DEBUG("Updating %s: %2.2f, %2.2f, %2.2f", tf_name.c_str(),
	    transform.getOrigin().x(), transform.getOrigin().y(), yaw);

  node_state[address].x = transform.getOrigin().x();
  node_state[address].y = transform.getOrigin().y();
  node_state[address].th = yaw;

  ROS_DEBUG("Checking %s: %2.2f, %2.2f, %2.2f", tf_name.c_str(),
	    node_state[address].x, node_state[address].y, node_state[address].th);

}

void RadioSim::SendPkt(int src, int dest, p2p_msgs::p2p &_msg)
{
  comms_request_reply_t ret;
  int retval;

  UpdateAgentState(src);

  // (1) Test send packet
  if(dest >= 0) {
    UpdateAgentState(dest);

    if(map_set) {
      vector<double> segments;
      path_sim.GetSignalPath(node_state[src].x, node_state[src].y, 0.0,
			     node_state[dest].x, node_state[dest].y, 0.0,
			     segments);
      retval = sim->TestSendPacket(src, dest, segments, &ret);
    }
    else {
      retval = sim->TestSendPacket(src, dest, &ret);
    }
    if(retval > 0) {
      _msg.rssi = ret.rssi;
      PublishPkt(dest, _msg);
    }
  }
  else {
    // Attempt Broadcast
    for(map<int, RadioSimNode*>::iterator i = this->node_objs.begin();
	i != this->node_objs.end();
	++i)
      {
	if(i->first != src) {
	  UpdateAgentState(i->first);

	  if(map_set) {
	    vector<double> segments;
	    path_sim.GetSignalPath(node_state[src].x, node_state[src].y, 0.0,
				   node_state[i->first].x, node_state[i->first].y, 0.0,
				   segments);
	    retval = sim->TestSendPacket(src, i->first, segments, &ret);
	  }
	  else {
	    retval = sim->TestSendPacket(src, i->first, &ret);
	  }
	  if(retval > 0) {
	    _msg.rssi = ret.rssi;
	    PublishPkt(i->first, _msg);
	  }
	}
      }
  }
}

void RadioSim::PublishPkt(int dest, const p2p_msgs::p2p &_msg)
{
  // (2) Push into recv node's publish queue
  if(node_objs.count(dest) == 0) {
    ROS_ERROR("Trying to send to non-existent node: %d", dest);

    ROS_DEBUG("Options are:");
    for(map<int, RadioSimNode*>::iterator i = this->node_objs.begin();
	i != this->node_objs.end();
	++i)
      {
	ROS_DEBUG("    %d (%s) ", i->first, i->second->name.c_str());
      }

    exit(-1);
  }

  if(dest < 0)
    ROS_ERROR("Implicitly creating node_object with address: %d", dest);

  p2p_msgs::p2p msg(_msg);
  msg.header.frame_id = node_objs[dest]->GetFrameID();
  node_objs[dest]->node_recv.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RadioSim::~RadioSim()
{

}

void RadioSim::handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  ROS_INFO("Loading map with resolution %f ...\n", msg->info.resolution);
  path_sim.LoadOccupancyGrid(*msg, path_sim_depth);
  ROS_INFO("done loading map\n");

  this->map_set = true;
}


RadioSimNode::RadioSimNode()
{
  ROS_ERROR("Tried to access/create a blank RadioSimNode!!!");
}

RadioSimNode::RadioSimNode(RadioSim *parent_, int address, const string &name, const string &radio_prefix)
{
  this->parent = parent_;
  this->address = address;
  this->name = name;
  this->node = ros::NodeHandle("~");

  string topic = string("/")+name+radio_prefix+string("/");

  node_send = node.subscribe(topic+"send", 10,
			     &RadioSimNode::OnP2PMessage, this);
  node_recv = node.advertise<p2p_msgs::p2p>(topic+"recv", 10);

  ROS_INFO_STREAM("Adding simulated radio: " << name << " (" << address << ") @ " << topic);
}

string RadioSimNode::GetFrameID()
{
  return string("/")+name+string("/base");
}

void RadioSimNode::OnP2PMessage(const p2p_msgs::p2pConstPtr &_msg)
{
  int encode_msglen = 0;

  p2p_msgs::p2p msg(*_msg);

  msg.src.resize(1);
  msg.header.frame_id = string("/")+name+string("/base");
  msg.header.stamp = ros::Time::now();
  msg.tx_power = this->parent->tx_power;
  msg.src[0] = this->address;

  if(msg.dest.size() == 0) {
    msg.dest.resize(1);
    msg.dest[0] = -1;
  }
  encode_msglen += msg.serializationLength();

  // TODO: Add packet splitting capabilities (ugh)
  if(encode_msglen > MAX_MESSAGE_SIZE) {
    ROS_ERROR("[RadioSim] message too big for packet!");
    return;
  }

  ROS_DEBUG("RadioSimSend P2P Message:");
  ROS_DEBUG("\tsrc=%d, dest=%d", msg.src[0], msg.dest[0]);
  ROS_DEBUG("\trssi: %2.2f, tx_power: %2.2f", msg.rssi, msg.tx_power);
  ROS_DEBUG("\t%d bytes of payload", msg.payload.size());
  if(msg.payload.size() > 0)
    ROS_DEBUG(": %d ...", msg.payload[0]);

  ROS_DEBUG("Trying to send packet to %d", msg.dest[0]);
  this->parent->SendPkt(msg.src[0], msg.dest[0], msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "radio_sim");
  ros::NodeHandle node;
  RadioSim radio;

  ros::spin();

  return 0;
}
