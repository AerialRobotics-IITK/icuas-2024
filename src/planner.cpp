#include "planner/planner.h"

planner::planner(ros::NodeHandle nh_, ros::Publisher traj_pub_) : nh(nh_), traj_pub(traj_pub_){
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

planner::~planner(){};

void planner::plan(void){

    std::vector<double> reals_goal;
    space->copyToReals(reals_goal, pdef->getGoal()->as<ob::GoalState>()->getState());
    std::cout << "Goal: " << "(" << reals_goal[0] << "," << reals_goal[1] << "," << reals_goal[2] << ")" << std::endl; 


    const ompl::base::State* statePtr = pdef->getStartState(0);
    const ompl::base::SE3StateSpace::StateType *state = statePtr->as<ompl::base::SE3StateSpace::StateType>();
    double start_x = state->getX();
    double start_y = state->getY();
    double start_z = state->getZ();
    std::cout << "Start: " << "("<< start_x << "," << start_y << "," << start_z << ")" << std::endl;

    
    ROS_INFO("Planning!");

    o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));
    o_plan->setProblemDefinition(pdef);
    o_plan->setup();

    pdef->print(std::cout);

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
    }
    else{
        ROS_INFO("No Solution Found");
    }
}

// void planner::setGoal(double x, double y, double z)
// {
//     ROS_INFO("Setting Goal!");
//     ob::ScopedState<ob::SE3StateSpace> goal(space);
//     goal->setXYZ(x, y, z);
//     goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

//     pdef->clearGoal();
//     pdef->setGoalState(goal);

//     ROS_INFO("(%f,%f,%f) set as goal!", x, y, z);

// }

// void planner::setStart(double x, double y, double z){
//     ROS_INFO("Resetting Start Position!");

//     ob::ScopedState<ob::SE3StateSpace> start(space);
//     start->setXYZ(x, y, z);
//     start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

//     pdef->clearStartStates();
//     pdef->addStartState(start);

//     ROS_INFO("(%f,%f,%f) set as start!", x, y, z);
// }

bool planner::isStateValid(const ob::State *state){
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

ob::OptimizationObjectivePtr planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr){
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    return obj;
}

void planner::run(std::vector<std::vector<double>> positions){

    std::vector<double> prev_pos = {1,1,1};
    for(auto pos : positions){
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(pos[0], pos[1], pos[2]);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(prev_pos[0], prev_pos[1], prev_pos[2]);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        pdef->clearSolutionPaths();  

        pdef->clearStartStates();
        pdef->addStartState(start);

        pdef->clearGoal();
        pdef->setGoalState(goal);

        plan();
        sleep(10); //wait at that position for a while

        prev_pos[0] = pos[0];
        prev_pos[1] = pos[1];
        prev_pos[2] = pos[2];
    }
}


