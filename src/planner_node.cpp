#include "planner/planner.h"

const std::string trajectory_topic = "/red/tracker/input_trajectory"; //TODO: create an yaml file for the topics and use nh.getparam to retrieve values
const std::string pose_topic = "/red/carrot/pose";

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Rate r(1);

    std::vector<std::vector<double>> positions;
    positions.push_back({7,10,4.5});
    positions.push_back({2,19,9});
    positions.push_back({10,19,9});
    //append waypoints here
    
    planner* planner_object = new planner(nh, r, trajectory_topic, pose_topic);   
    planner_object->run(positions);

}