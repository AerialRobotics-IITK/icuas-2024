#include "intermediate.hpp"

const std::string trajectory_cmd_topic = "/red/tracker/input_trajectory";

int main(int argc, char** argv) {
  ros::init(argc, argv, "intermediate_node");
  ros::NodeHandle nh; 

  std::vector<WaypointWithTime> waypoints{
      WaypointWithTime(10, 1, 5, 2, M_PI/2),
      WaypointWithTime(10, 10, 6, 5, -M_PI/2),
      WaypointWithTime(10, 10, 10, 3, 0),
      WaypointWithTime(10, 1, 1, 1, M_PI)
  };
  
  WaypointPublisher publisher(nh, trajectory_cmd_topic, waypoints);
  publisher.run();

  ros::shutdown();
  return 0;
}