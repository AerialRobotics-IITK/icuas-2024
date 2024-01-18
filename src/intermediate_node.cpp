#include "intermediate.hpp"

const std::string trajectory_topic = "/red/tracker/input_trajectory";
const std::string position_topic = "/red/carrot/pose";
const std::string count_topic = "/fruit_count";

int main(int argc, char** argv) {
  ros::init(argc, argv, "intermediate_node");
  ros::NodeHandle nh; 
  ros::Rate r(10);

  std::vector<WaypointWithTime> waypoints{
      WaypointWithTime(10, 1, 5, 2, M_PI/2),
      WaypointWithTime(10, 10, 6, 5, -M_PI/2),
      WaypointWithTime(10, 10, 10, 3, 0),
      WaypointWithTime(10, 1, 1, 1, M_PI)
  };
  
  WaypointPublisher publisher(nh, r, trajectory_topic, position_topic, count_topic, waypoints);
  publisher.run();

  ros::shutdown();
  return 0;
}