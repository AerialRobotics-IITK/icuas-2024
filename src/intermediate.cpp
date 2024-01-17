#include "intermediate.hpp";


WaypointPublisher::WaypointPublisher(ros::NodeHandle nh, std::string trajectory_topic, std::vector<WaypointWithTime> _waypoints) {
    ROS_INFO("Running waypoint publisher.");
    wp_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(trajectory_topic, 10);
    waypoints = _waypoints;
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

    ROS_INFO("Started publishing waypoints.");
    wp_pub.publish(msg);       
    ros::spinOnce();
}

WaypointPublisher::~WaypointPublisher(){
}
