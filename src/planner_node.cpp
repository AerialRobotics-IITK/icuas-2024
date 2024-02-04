#include "planner/planner.h"

const std::string trajectory_topic = "/red/tracker/input_trajectory";

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(trajectory_topic, 1); 

    std::vector<std::vector<double>> positions;
    positions.push_back({7,12,4.5});
    positions.push_back({5,1,14});
    
    planner* planner_object = new planner(nh, traj_pub);   
    planner_object->run(positions);
    // ros::spin();
}