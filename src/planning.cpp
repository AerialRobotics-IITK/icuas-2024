#include <ros/ros.h>

#include "fcl/config.h"
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <Eigen/Geometry>

#include <std_msgs/Int32.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <iostream>
#include <utils/utils.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
constexpr int64_t kNanoSecondsInSecond = 1000000000;

ros::Publisher traj_pub;
const std::string trajectory_topic = "/red/tracker/input_trajectory";

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
public:
    planner(void){
        aircraftObject = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(1.5, 1.5, 1.5)));
        shelfOne = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(2.0, 21.0, 16.0)));
        shelfTwo = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(2.0, 21.0, 16.0)));
        shelfThree = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(2.0, 21.0, 16.0)));

        space = ob::StateSpacePtr(new ob::SE3StateSpace());

        ob::ScopedState<ob::SE3StateSpace> start(space);
        ob::ScopedState<ob::SE3StateSpace> goal(space);
    
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0,0);  //bounds for x-axis
        bounds.setHigh(0,30);        
        bounds.setLow(1,0); //bounds for y-axis
        bounds.setHigh(1,30);
        bounds.setLow(2, 0); //bounds for z-axis
        bounds.setHigh(2,16);
        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        start->setXYZ(1,1,1);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        goal->setXYZ(0,0,0);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

        o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));
        o_plan->setProblemDefinition(pdef);
        o_plan->setup();
        
        ROS_CYAN_STREAM("Planner Initialized");
        pdef->print(std::cout);
    }
    
    ~planner(){};

    void plan(void){
        ROS_INFO("Planning!");
        ob::PlannerStatus solved = o_plan->solve(4);
        if(solved){
            ROS_INFO("Found Solution:");

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            pth->printAsMatrix(std::cout);

            trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
            msg->header.stamp = ros::Time::now();
            msg->points.resize(pth->getStateCount());
            ROS_INFO("Resized message to hold %d point(s) in the message", pth->getStateCount());

            msg->joint_names.push_back("base_link");

            int64_t time_from_start_ns = 0;
            for (size_t i = 0; i < pth->getStateCount(); ++i) {
                const ob::SE3StateSpace::StateType *se3state = pth->getState(i)->as<ob::SE3StateSpace::StateType>();
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                Eigen::Vector3d p(pos->values[0], pos->values[1], pos->values[2]);
                ROS_INFO("Set position vector");
                Eigen::Quaterniond quat(rot->w, rot->x, rot->y, rot->z);
                ROS_INFO("Set Quaternion");

                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = p;
                trajectory_point.orientation_W_B = quat;
                trajectory_point.time_from_start_ns = time_from_start_ns;

                time_from_start_ns += static_cast<int64_t>(4 * kNanoSecondsInSecond);

                mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
                ROS_INFO("%d : trajectory point set", i);
            }
            ROS_CYAN_STREAM("Started publishing waypoints.");
            traj_pub.publish(msg);       
            ros::spinOnce();

        }
        else{
            ROS_INFO("No Solution Found");
        }
    }

	void setGoal(double x, double y, double z)
	{
        ROS_INFO("Setting Goal!");
		ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(x, y, z);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        pdef->clearGoal();
        pdef->setGoalState(goal);
        // ob::State *state =  space->allocState();
        
        // state->as<ob::RealVectorStateSpace::StateType>()->values = goal->values;
        ROS_INFO("(%f,%f,%f) set as goal!", x, y, z);

        plan();
	}

    bool isStateValid(const ob::State *state){
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        fcl::Vector3<double> aircraftTranslation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaternion<double> aircraftRotation(rot->w, rot->x, rot->y, rot->z);
        
        fcl::Vector3<double> shelfOneTranslation(4, 13.5, 0.5);
        fcl::Quaternion<double> shelfOneRotation(1, 0, 0, 0);
    
        fcl::Vector3<double> shelfTwoTranslation(10, 13.5, 0.5);
        fcl::Quaternion<double> shelfTwoRotation(1, 0, 0, 0);

        fcl::Vector3<double> shelfThreeTranslation(16, 13.5, 0.5);
        fcl::Quaternion<double> shelfThreeRotation(1, 0, 0, 0);

        aircraftObject->setTransform(aircraftRotation, aircraftTranslation);
        shelfOne->setTransform(shelfOneRotation, shelfOneTranslation);
        shelfTwo->setTransform(shelfTwoRotation, shelfTwoTranslation);
        shelfThree->setTransform(shelfThreeRotation, shelfThreeTranslation);

        fcl::CollisionRequest<double> requestType(1,false,1,false);

        fcl::CollisionResult<double> collisionResultShelfOne;
        fcl::collide(aircraftObject.get(), shelfOne.get(), requestType, collisionResultShelfOne);
        
        fcl::CollisionResult<double> collisionResultShelfTwo;
        fcl::collide(aircraftObject.get(), shelfTwo.get(), requestType, collisionResultShelfTwo);

        fcl::CollisionResult<double> collisionResultShelfThree;
        fcl::collide(aircraftObject.get(), shelfThree.get(), requestType, collisionResultShelfThree);
    
        return!( collisionResultShelfOne.isCollision() || collisionResultShelfTwo.isCollision() || collisionResultShelfThree.isCollision() );
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr){
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        return obj;
    }
};

int main(int argc, char **argv){
    //planner_object;
    planner* planner_object = new planner;
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    
    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(trajectory_topic, 1);
    planner_object->setGoal(7,12,4.5);

    // ros::spin();
}