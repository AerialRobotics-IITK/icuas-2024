#include "intermediate/intermediate.hpp"

const std::string trajectory_topic = "/red/tracker/input_trajectory";
const std::string position_topic = "/red/carrot/pose";
const std::string count_topic = "/fruit_count";

int main(int argc, char** argv) {
  ros::init(argc, argv, "intermediate_node");
  ros::NodeHandle nh; 
  ros::Rate r(10);

  //harcoded trajectory to traverse all shelves
  /*
  std::vector<WaypointWithTime> waypoints{
      WaypointWithTime(1, 1, 1, 2, 0),
      WaypointWithTime(30, 1, 24, 2, 0),
      WaypointWithTime(8,1, 24, 4.5, 0),
      WaypointWithTime(30, 1, 1, 4.5, 0),
      WaypointWithTime(8, 1, 1, 8, 0),
      WaypointWithTime(30, 1, 24, 8, 0),
      WaypointWithTime(8,1,24,10,0),
      

      WaypointWithTime(1,7,24,8,M_PI),
      WaypointWithTime(30,7,1,8,M_PI),
      WaypointWithTime(8,7,1,4.5,M_PI),
      WaypointWithTime(30,7,24,4.5,M_PI),
      WaypointWithTime(8,7,24,2,M_PI),
      WaypointWithTime(30,7,1,2,M_PI),
      
      WaypointWithTime(30,7,1,2,0),
      WaypointWithTime(30,7,24,2,0),
      WaypointWithTime(8,7,24,4.5,0),
      WaypointWithTime(30,7,1,4.5,0),
      WaypointWithTime(8,7,1,8,0),
      WaypointWithTime(30,7,24,8,0),
      
      WaypointWithTime(8,7,24,10,M_PI),
      WaypointWithTime(1,13,24,8,M_PI),
      WaypointWithTime(30,13,1,8,M_PI),
      WaypointWithTime(8,13,1,4.5,M_PI),
      WaypointWithTime(30,13,24,4.5,M_PI),
      WaypointWithTime(8,13,24,2,M_PI),
      WaypointWithTime(30,13,1,2,M_PI),
      

      WaypointWithTime(30,13,1,2,0),
      WaypointWithTime(30,13,24,2,0),
      WaypointWithTime(8,13,24,4.5,0),
      WaypointWithTime(30,13,1,4.5,0),
      WaypointWithTime(8,13,1,8,0),
      WaypointWithTime(30,13,24,8,0),

      WaypointWithTime(8,13,24,10,M_PI),
      WaypointWithTime(30,19,24,8,M_PI),
      WaypointWithTime(30,19,1,8,M_PI),
      WaypointWithTime(8,19,1,4.5,M_PI),
      WaypointWithTime(30,19,24,4.5,M_PI),
      WaypointWithTime(8,19,24,2,M_PI),
      WaypointWithTime(30,19,1,2,M_PI),

  };
  */

  std::vector<WaypointWithTime> waypoints{
      WaypointWithTime(1, 1, 1, 1, 0)
  };

  
  WaypointPublisher publisher(nh, r, trajectory_topic, position_topic, count_topic, waypoints);
  publisher.run();

  ros::shutdown();
  return 0;
}