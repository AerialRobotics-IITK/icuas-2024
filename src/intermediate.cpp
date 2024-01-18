#include "intermediate.hpp"
#include "utils.hpp"


/*helper functions*/

_Float64 distance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) {
    return pow(pow(p1.pose.position.x-p2.pose.position.x, 2) + pow(p1.pose.position.y-p2.pose.position.y, 2) + pow(p1.pose.position.z-p2.pose.position.z, 2), 0.5);
}

geometry_msgs::PoseStamped wpToPoseStamped(WaypointWithTime wp){
    geometry_msgs::PoseStamped _pose;
    _pose.pose.position.x = wp.position[0];
    _pose.pose.position.y = wp.position[1];
    _pose.pose.position.z = wp.position[2];

    return _pose;
}


/*waypoint publisher class definitions*/

WaypointPublisher::WaypointPublisher(ros::NodeHandle _nh, ros::Rate _r, 
    std::string _trajectory_topic, std::string _position_topic, std::string _count_topic,
    std::vector<WaypointWithTime> _waypoints) : nh(_nh), r(_r), waypoints(_waypoints){

    ROS_CYAN_STREAM("Running waypoint publisher.");
    wp_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(_trajectory_topic, 10);
    ref_sub = nh.subscribe(_position_topic, 100, &WaypointPublisher::positionCb, this);

    count_pub = nh.advertise<std_msgs::Int32>(_count_topic, 10);
}

void WaypointPublisher::run(){
    ros::Duration(5).sleep();

    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    msg->header.stamp = ros::Time::now();
    msg->points.resize(waypoints.size());
    msg->joint_names.push_back("base_link");
    int64_t time_from_start_ns = 0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        WaypointWithTime& wp = waypoints[i];

        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = wp.position;
        trajectory_point.setFromYaw(wp.yaw);
        trajectory_point.time_from_start_ns = time_from_start_ns;

        time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
    }

    ROS_CYAN_STREAM("Started publishing waypoints.");
    wp_pub.publish(msg);       
    ros::spinOnce();
    

    for(int i = 0; i < waypoints.size(); i++){

        geometry_msgs::PoseStamped target_position = wpToPoseStamped(waypoints[i]);
        _Float64 current_distance = distance(target_position, current_position);

        while( ros::ok() && distance(target_position, current_position) > tolerance ) {

            ROS_INFO("Target position = (%f, %f, %f) Current position = (%f, %f, %f) Distance = %f", 
                target_position.pose.position.x, target_position.pose.position.y, target_position.pose.position.z, 
                current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, 
                current_distance);

            current_distance = distance(target_position, current_position);

            ros::spinOnce();
            r.sleep();
        }
        ROS_INFO("Reached target waypoint = (%f, %f, %f)", 
            target_position.pose.position.x, target_position.pose.position.y, target_position.pose.position.z);
    }

    ROS_GREEN_STREAM("Reached starting point = (1, 1, 1)");


    ROS_CYAN_STREAM("Started publishing fruit count");
    std_msgs::Int32 fruit_count;
    fruit_count.data = 42;

    while(ros::ok()){
        count_pub.publish(fruit_count);
        r.sleep();
    }
    
}

void WaypointPublisher::positionCb(const geometry_msgs::PoseStamped msg){
    current_position = msg;
}   

WaypointPublisher::~WaypointPublisher(){
}

