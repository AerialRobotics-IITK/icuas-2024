#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <std_msgs/Int32.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

constexpr int64_t kNanoSecondsInSecond = 1000000000;

struct WaypointWithTime {
 public:
  Eigen::Vector3d position;
  double yaw;
  double waiting_time;

  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }
};

class WaypointPublisher{
private:
    ros::NodeHandle nh;
    ros::Rate r;

    ros::Publisher wp_pub;
    ros::Subscriber ref_sub;
    ros::Publisher count_pub;
    std::vector<WaypointWithTime> waypoints;

    const double tolerance = 0.1;

public:
    geometry_msgs::PoseStamped current_position;

    WaypointPublisher(ros::NodeHandle nh, ros::Rate r, 
    std::string trajectory_topic, std::string position_topic, std::string count_topic,
    std::vector<WaypointWithTime> _waypoints);

    void positionCb(const geometry_msgs::PoseStamped);
    ~WaypointPublisher();
    void run();
};
