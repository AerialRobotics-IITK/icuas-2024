#include "planner/planner.h"
// #include "trajectory/trajectory_generation_new.hpp"

const std::string trajectory_topic = "/red/tracker/input_trajectory"; //TODO: create an yaml file for the topics and use nh.getparam to retrieve values
const std::string pose_topic = "/red/carrot/pose";

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Rate r(1);
    //generate trajectory
    // std::vector<int> points={1,5,2,6,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
    // Trajectory path(1,1,10,points);

    
    std::vector<std::vector<double>> positions;

    // positions.push_back({8.00,4.50,1.10});
    // positions.push_back({8.0,5.00,1.10});
    // positions.push_back({8.0,6.50,1.10});

    // positions.push_back({8.0,4.50,3.9});
    // positions.push_back({8.0,5.00,3.9});
    // positions.push_back({8.0,6.50,3.9});

    // positions.push_back({8.0,7.0,3.9});
    // positions.push_back({8.0,8.5,3.9});
    // positions.push_back({8.0,9.0,3.9});

    // positions.push_back({14.0,4.5,6.7});
    // positions.push_back({14.0,6.0,6.7});
    // positions.push_back({14.0,7.5,6.7}); 

    // positions.push_back({14.0,7.0,3.9});
    // positions.push_back({14.0,8.5,3.9});
    // positions.push_back({14.0,9.0,3.9});

    // positions.push_back({14.0,7.0,6.7});
    // positions.push_back({14.0,8.5,6.7});
    // positions.push_back({14.0,9.0,6.7});

    //append waypoints here
    
    planner* planner_object = new planner(nh, r, trajectory_topic, pose_topic);   
    planner_object->run(positions);

}