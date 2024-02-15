#include <ros/ros.h>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

//Planners
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include <Eigen/Geometry>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <future>
#include <unistd.h>

#include <boost/math/quaternion.hpp>
#include <boost/math/constants/constants.hpp>

#include "utils/utils.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;
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

class planner{
private:
    ob::StateSpacePtr space;
    ob::SpaceInformationPtr si;
    ob::ProblemDefinitionPtr pdef;

    og::PathGeometric* path_smooth = NULL;
    
    ob::PlannerPtr o_plan;

	std::shared_ptr<fcl::CollisionObject<double>> aircraftObject;
	std::shared_ptr<fcl::CollisionObject<double>> shelfOne;
	std::shared_ptr<fcl::CollisionObject<double>> shelfTwo;
	std::shared_ptr<fcl::CollisionObject<double>> shelfThree;

    ros::NodeHandle nh;
    ros::Publisher traj_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber plant_sub;
    ros::Publisher scan_flag_pub;
    ros::Rate r;

    //initalized initial current goals to (1,1,1)
    double curr_x = 1;
    double curr_y = 1;
    double curr_z = 1;
    double curr_yaw = 0;

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr);
    bool isStateValid(const ob::State *state);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
    void plantCallback(const std_msgs::String::ConstPtr& plantMsg);
    double getDistance(double x, double y, double z);
public:
    planner(ros::NodeHandle nh_, ros::Rate r_, std::string trajectory_topic_, std::string pose_topic_, std::string plant_topic_, std::string scan_flag_topic_);
    ~planner();

    std::vector<int> plant_beds;

    void plan(void);
    void run(std::vector<std::vector<double>> positions);
};