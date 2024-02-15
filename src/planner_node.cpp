#include "planner/planner.h"
#include "planner/permute_traj.h"
#define HARDCODE_POINTS 0

const unsigned int microsecond = 1000000;

const std::string trajectory_topic = "/red/tracker/input_trajectory"; //TODO: create an yaml file for the topics and use nh.getparam to retrieve values
const std::string pose_topic = "/red/carrot/pose";
const std::string plant_topic = "/red/plants_beds";
const std::string count_topic = "/fruit_count";
const std::string scan_flag_topic = "/scan";
const std::string trajectory_flag_topic = "/in_trajectory";

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Rate r(10);

    planner* planner_object = new planner(nh, r, trajectory_topic, pose_topic, plant_topic, scan_flag_topic);   
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
    trajectory_gen* traj_generator = new trajectory_gen(6, 7.7, 2.8, 4, 5.8, 1.35, plant_positions); //offset values and x,y,z are hardcoded
    positions = traj_generator->get_final_waypoints();
    positions.push_back({1, 1, 1, 3.1452});
#endif
    //defining home traj msg
    ros::Publisher wp_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(trajectory_topic, 10);

    trajectory_msgs::MultiDOFJointTrajectoryPtr home_msg(new trajectory_msgs::MultiDOFJointTrajectory);
    home_msg->header.stamp = ros::Time::now();
    home_msg->points.resize(1);
    home_msg->joint_names.push_back("base_link");

    WaypointWithTime home_wp = WaypointWithTime(8, 1, 1, 1, 0);

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = home_wp.position;
    trajectory_point.setFromYaw(home_wp.yaw);
    trajectory_point.time_from_start_ns = 0;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &home_msg->points[0]);


    /*start wp to topra*/
    wp_pub.publish(home_msg);  
    usleep(2 * microsecond);

    //running planner
    planner_object->run(positions);
    ROS_CYAN_STREAM("Mission Completed!");

    ros::Publisher trajectory_flag_pub;
    trajectory_flag_pub = nh.advertise<std_msgs::Bool>(trajectory_flag_topic, 10);

    std_msgs::Int32 trajectory_flag;
    trajectory_flag.data = false;

    usleep(2 * microsecond);

    while(ros::ok()){
        usleep(2 * microsecond);//sleeps for 2 second
        ROS_YELLOW_STREAM("Sending end trajectory flag");
        trajectory_flag_pub.publish(trajectory_flag); 
        r.sleep();
    }
}