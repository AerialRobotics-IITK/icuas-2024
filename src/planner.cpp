#include "planner/planner.h"

#define DEBUG 1
#define TAKE_YAW_AS_INPUT 1

planner::planner(ros::NodeHandle nh_, ros::Rate r_, std::string trajectory_topic_, std::string pose_topic_, std::string plant_topic_, std::string scan_flag_topic_) : nh(nh_), r(r_){

    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(trajectory_topic_, 10);
    pos_sub = nh.subscribe(pose_topic_, 10, &planner::poseCallback, this);
    plant_sub = nh.subscribe(plant_topic_, 10, &planner::plantCallback, this);

    scan_flag_pub = nh.advertise<std_msgs::Bool>(scan_flag_topic_, 10); 

    aircraftObject = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(1.5, 1.5, 1.5)));
    shelfOne = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(2.3, 21.0, 20)));
    shelfTwo = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(2.3, 21.0, 20)));
    shelfThree = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(2.3, 21.0,20)));

    space = ob::StateSpacePtr(new ob::SE3StateSpace());

    ob::ScopedState<ob::SE3StateSpace> start(space);
    ob::ScopedState<ob::SE3StateSpace> goal(space);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,0);  //bounds for x-axis
    bounds.setHigh(0,30);        
    bounds.setLow(1,0); //bounds for y-axis
    bounds.setHigh(1,30);
    bounds.setLow(2, 0); //bounds for z-axis
    bounds.setHigh(2,10);
    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // initializing waypoints
    start->setXYZ(this->curr_x,this->curr_y,this->curr_z);
    
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    goal->setXYZ(1,1,1);
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    // defining validity of state
    si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));

    // defining problem definition on the state space described
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);
    //TODO: backtest different optimization objectives
    pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

    // defining planner; right now it uses a sampling-based approach i.e. RRT 
    //o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));
    //o_plan = ob::PlannerPtr(new og::RRTConnect(si)); // Using RRTConnect planner  
    o_plan = ob::PlannerPtr(new og::RRT(si)); // Using RRT planner (or another deterministic planner of your choice)  
    o_plan->setProblemDefinition(pdef);
    o_plan->setup();
    
    r.sleep();
    ros::spinOnce();

    ROS_CYAN_STREAM("Planner Initialized");

#if DEBUG
    pdef->print(std::cout);
#endif

}

planner::~planner(){};

void planner::plan(void){


    /*Getting and priniting values at runtime for debugging*/
#if DEBUG
    std::vector<double> reals_goal;
    space->copyToReals(reals_goal, pdef->getGoal()->as<ob::GoalState>()->getState());
    std::cout << "Goal: " << "(" << reals_goal[0] << "," << reals_goal[1] << "," << reals_goal[2] << ")" << std::endl; 


    const ompl::base::State* statePtr = pdef->getStartState(0);
    const ompl::base::SE3StateSpace::StateType *state = statePtr->as<ompl::base::SE3StateSpace::StateType>();
    double start_x = state->getX();
    double start_y = state->getY();
    double start_z = state->getZ();
    std::cout << "Start: " << "("<< start_x << "," << start_y << "," << start_z << ")" << std::endl;
#endif
    
    ROS_INFO("Planning!");

    // initializing planner again, to reflect changes in start and endpoint
    o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));
    o_plan->setProblemDefinition(pdef);
    o_plan->setup();

#if DEBUG
    pdef->print(std::cout);
#endif 

    ob::PlannerStatus solved = o_plan->solve(1.5);
    if(solved){
        ROS_INFO("Found Solution:");

        ob::PathPtr path = pdef->getSolutionPath();
        og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();

        //printing the solution path
        pth->printAsMatrix(std::cout);

        //creating the trajectory msg of the generating path for topra
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
        //invalid waypoint given; can be due to being defined outside bounds or because it is colliding with the collision geometries
        ROS_INFO("No Solution Found");
    }
}

bool planner::isStateValid(const ob::State *state){
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    fcl::Vector3<double> aircraftTranslation(pos->values[0],pos->values[1],pos->values[2]);
    fcl::Quaternion<double> aircraftRotation(rot->w, rot->x, rot->y, rot->z);
    
    fcl::Vector3<double> shelfOneTranslation(4, 13.5, 0);
    fcl::Quaternion<double> shelfOneRotation(1, 0, 0, 0);

    fcl::Vector3<double> shelfTwoTranslation(10, 13.5, 0);
    fcl::Quaternion<double> shelfTwoRotation(1, 0, 0, 0);

    fcl::Vector3<double> shelfThreeTranslation(16, 13.5, 0);
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

void planner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
#if DEBUG
    ROS_INFO("Recieved Pose Info");
#endif
    this->curr_x = poseMsg->pose.position.x;
    this->curr_y = poseMsg->pose.position.y;
    this->curr_z = poseMsg->pose.position.z;
}

void planner::plantCallback(const std_msgs::String::ConstPtr& plantMsg){
#if DEBUG
    ROS_INFO("Recieved Plant Info");
#endif
/*
1. Tomato: red
2. Pepper: yellow
3. Eggplant: purple
*/
    std::string data = plantMsg->data;
    auto res = util::split(data, " ");

    this->plant_beds = res.second;
    ROS_INFO("Looking for plant: %s!", res.first.c_str());
}

double planner::getDistance(double x, double y, double z) {
    return pow(pow(x - this->curr_x, 2) + pow(y - this->curr_y, 2) + pow(z - this->curr_z, 2), 0.5);
}

void planner::run(std::vector<std::vector<double>> positions){

#if TAKE_YAW_AS_INPUT
    std::vector<double> prev_pos = {this->curr_x, this->curr_y, this->curr_z, 0};
    util::Quaternion quat;
    util::Quaternion prev_quat;

    std_msgs::Bool scan_flag;

    for(auto pos : positions){
        while(getDistance(prev_pos[0], prev_pos[1], prev_pos[2]) > 0.1){
            scan_flag.data = false;
            ROS_INFO("On way to current waypoint (%f, %f, %f, %f)", prev_pos[0], prev_pos[1], prev_pos[2], prev_pos[3]);
            scan_flag_pub.publish(scan_flag);
            ros::spinOnce();
            r.sleep();
        }
        
        int k = 20;
        while(k--){
            scan_flag.data = true;
            scan_flag_pub.publish(scan_flag);
        }

        quat = util::rpyToQuaternion(0, 0, pos[3]);
        prev_quat = util::rpyToQuaternion(0, 0, prev_pos[3]);

        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(pos[0], pos[1], pos[2]);
        goal->as<ob::SO3StateSpace::StateType>(1)->x = quat.x;
        goal->as<ob::SO3StateSpace::StateType>(1)->y = quat.y;
        goal->as<ob::SO3StateSpace::StateType>(1)->z = quat.z;
        goal->as<ob::SO3StateSpace::StateType>(1)->w = quat.w;

        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(prev_pos[0], prev_pos[1], prev_pos[2]);
        start->as<ob::SO3StateSpace::StateType>(1)->x = prev_quat.x;
        start->as<ob::SO3StateSpace::StateType>(1)->y = prev_quat.y;
        start->as<ob::SO3StateSpace::StateType>(1)->z = prev_quat.z;
        start->as<ob::SO3StateSpace::StateType>(1)->w = prev_quat.w;

        // clearing the stored solution paths / waypoints and assigning new ones
        pdef->clearSolutionPaths();  

        pdef->clearStartStates();
        pdef->addStartState(start);

        pdef->clearGoal();
        pdef->setGoalState(goal);

        plan();

        prev_pos[0] = pos[0];
        prev_pos[1] = pos[1];
        prev_pos[2] = pos[2];
        prev_pos[3] = pos[3];

        this->curr_x = pos[0];
        this->curr_y = pos[1];
        this->curr_z = pos[2];

        scan_flag.data = true;
        scan_flag_pub.publish(scan_flag);

        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Reaching last waypoint (%f, %f, %f). Exiting planner!", this->curr_x, this->curr_y, this->curr_z);

#else 
/*rotation matrix is hardcoded as identity i.e. rpy = (0,0,0)*/

    std::vector<double> prev_pos = {this->curr_x, this->curr_y, this->curr_z};
    for(auto pos : positions){
        while(getDistance(prev_pos[0], prev_pos[1], prev_pos[2]) > 0.1){
            ROS_INFO("On way to current waypoint (%f, %f, %f)", prev_pos[0], prev_pos[1], prev_pos[2]);

            ros::spinOnce();
            r.sleep();
        }

        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(pos[0], pos[1], pos[2]);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(prev_pos[0], prev_pos[1], prev_pos[2]);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        // clearing the stored solution paths / waypoints and assigning new ones
        pdef->clearSolutionPaths();  

        pdef->clearStartStates();
        pdef->addStartState(start);

        pdef->clearGoal();
        pdef->setGoalState(goal);

        plan();

        prev_pos[0] = pos[0];
        prev_pos[1] = pos[1];
        prev_pos[2] = pos[2];

        this->curr_x = pos[0];
        this->curr_y = pos[1];
        this->curr_z = pos[2];

        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Reaching last waypoint (%f, %f, %f). Exiting planner!", this->curr_x, this->curr_y, this->curr_z);
#endif

}


