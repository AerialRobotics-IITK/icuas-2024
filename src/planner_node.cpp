#include "planner/planner.h"
#include "planner/permute_traj.h"
#define HARDCODE_POINTS 0

const std::string trajectory_topic = "/red/tracker/input_trajectory"; //TODO: create an yaml file for the topics and use nh.getparam to retrieve values
const std::string pose_topic = "/red/carrot/pose";
const std::string plant_topic = "/red/plants_beds";

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Rate r(50);

    planner* planner_object = new planner(nh, r, trajectory_topic, pose_topic, plant_topic);   
    auto plant_positions = planner_object->plant_beds;

    ROS_YELLOW_STREAM("Plant positions:");
    for(int i = 0; i < 100; i++)
    {
        for(auto it : plant_positions){
            std::cout << it << " ";
        }
        std::cout << std::endl;
    }


    /*Defining goals*/
    std::vector<std::vector<double>> positions;
#if HARDCODE_POINTS 
    /*hardcoded waypoints for debugging*/
    positions.push_back({8.00,4.50,1.10});
    positions.push_back({8.0,5.00,1.10});
    positions.push_back({8.0,6.50,1.10});

    positions.push_back({8.0,4.50,3.9});
    positions.push_back({8.0,5.00,3.9});
    positions.push_back({8.0,6.50,3.9});

    positions.push_back({8.0,7.0,3.9});
    positions.push_back({8.0,8.5,3.9});
    positions.push_back({8.0,9.0,3.9});

    positions.push_back({14.0,4.5,6.7});
    positions.push_back({14.0,6.0,6.7});
    positions.push_back({14.0,7.5,6.7}); 

    positions.push_back({14.0,7.0,3.9});
    positions.push_back({14.0,8.5,3.9});
    positions.push_back({14.0,9.0,3.9});

    positions.push_back({14.0,7.0,6.7});
    positions.push_back({14.0,8.5,6.7});
    positions.push_back({14.0,9.0,6.7});
    //append waypoints here
#else
    trajectory_gen* traj_generator = new trajectory_gen(6, 7.7, 2.8, 4, 6, 1.35, plant_positions); //offset values and x,y,z are hardcoded
    positions = traj_generator->get_final_waypoints();
#endif


    //running planner
    planner_object->run(positions);
    while(ros::ok()){
        ROS_CYAN_STREAM("Mission Completed!");
        //wait indefinitely
        std::promise<void>().get_future().wait();
    }
}