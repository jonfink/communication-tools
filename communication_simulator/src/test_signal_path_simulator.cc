#include <iostream>
#include <vector>
#include <string.h>

#include <stdlib.h>
#include <stdio.h>

#include "SignalPathSimulator.h"

SignalPathSimulator sim;

bool map_set = false;
double depth = 0;
void handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  printf("Loading map with resolution %f ...\n", msg->info.resolution);
  sim.LoadOccupancyGrid(*msg, depth);
  printf("done\n");

  map_set = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_signal_path_simulator");
  ros::NodeHandle n("~");

  ros::Subscriber map_sub = n.subscribe("/map", 1, handle_map); 

  n.param("depth", depth, 0.5);

  ros::Rate r(1);
  
  while(ros::ok()) {
    ros::spinOnce();

    if(map_set) {
      std::vector<double> segments;
      sim.GetSignalPath(0.0, 0.0, 0.0, 0.0, 6.0, 0.0, segments);

      printf("[");
      for(unsigned int i=0; i < segments.size(); ++i) {
	printf("%2.2f ", segments[i]);
      }
      printf("]\n");
	
    }
    r.sleep();
  }

  return 0;
}
