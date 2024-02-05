#include <ros/ros.h>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include <Eigen/Geometry>

#include <std_msgs/Int32.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <iostream>
#include <vector>

#include "utils/utils.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;
constexpr int64_t kNanoSecondsInSecond = 1000000000;

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

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr);
    bool isStateValid(const ob::State *state);
public:
    planner(ros::NodeHandle nh_, ros::Publisher traj_pub_);
    ~planner();

    void plan(void);
	void setGoal(double x, double y, double z);
    void setStart(double x, double y, double z);
    void run(std::vector<std::vector<double>> positions);
};