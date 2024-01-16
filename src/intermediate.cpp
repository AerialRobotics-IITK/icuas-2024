#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <math.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace mavros_msgs;

typedef struct QuaternionCustom QuaternionCustom;

struct QuaternionCustom {
    double w, x, y, z;
};

QuaternionCustom ToQuaternion(double roll, double pitch, double yaw) {
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    QuaternionCustom q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


bool isConnected = false, isArmed = false, isGuided = false;
PoseStamped current_position;

_Float64 distance(PoseStamped p1, PoseStamped p2) {
    return pow(pow(p1.pose.position.x-p2.pose.position.x, 2) + pow(p1.pose.position.y-p2.pose.position.y, 2) + pow(p1.pose.position.z-p2.pose.position.z, 2), 0.5);
}
PoseStamped setPose(_Float64 x, _Float64 y, _Float64 z, _Float64 yaw = 0, _Float64 roll = 0, _Float64 pitch = 0) {
    PoseStamped position;

    QuaternionCustom yaw_data = ToQuaternion(roll, pitch, yaw);
    position.pose.position.x = x;
    position.pose.position.y = y;
    position.pose.position.z = z;

    position.pose.orientation.x = (_Float64) yaw_data.x;
    position.pose.orientation.y = (_Float64) yaw_data.y;
    position.pose.orientation.z = (_Float64) yaw_data.z;
    position.pose.orientation.w = (_Float64) yaw_data.w;

    return position;
}
void goToPose(PoseStamped target_position, Publisher position_publisher, float tolerance, Rate rate) {
    position_publisher.publish(target_position);
    _Float64 current_distance = distance(target_position, current_position);
    while( ok() && distance(target_position, current_position) > tolerance ) {
        ROS_INFO("Target Position = (%f, %f, %f) Current Position = (%f, %f, %f) Distance = %f", target_position.pose.position.x, target_position.pose.position.y, target_position.pose.position.z, current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, current_distance);
        current_distance = distance(target_position, current_position);
        spinOnce();
        rate.sleep();
    }
}

void getState(State msg) {
    isConnected = msg.connected;
    isArmed = msg.armed;
    if( msg.mode == "GUIDED" )
        isGuided = true;
    else
        isGuided = false;
}
void updatePosition(PoseStamped msg) {
    current_position = msg;
}

int main(int argc, char *argv[]) {
    int rate, queue_size;
    float altitude, tolerance;
    string set_mode_service, arm_service, takeoff_service, land_service;
    string state_subscription, position_subscription;
    string position_publisher;
    string mode;
    string topic_name;
    SetMode mode_command;
    CommandBool arm_command;
    CommandTOL takeoff_command, land_command;

    init(argc, argv, "icuas24");

    NodeHandle node("~");
    node.getParam("rate", rate);
    node.getParam("queue_size", queue_size);
    node.getParam("altitude", altitude);
    node.getParam("tolerance", tolerance);
    node.getParam("mode", mode);
    node.getParam("set_mode_service", set_mode_service);
    node.getParam("arm_service", arm_service);
    node.getParam("takeoff_service", takeoff_service);
    node.getParam("land_service", land_service);
    node.getParam("state_subscription", state_subscription);
    node.getParam("position_subscription", position_subscription);
    node.getParam("position_publisher", position_publisher);
    node.getParam("topic_name", topic_name);

    PoseStamped initialPosition = setPose(0, 0, altitude);
    PoseStamped startPosition = setPose(1, 1, altitude);

    // Add Points for Path
    vector<PoseStamped> path;
    path.push_back(setPose(1, 5, 2, (_Float64) M_PI/2));
    path.push_back(setPose(10, 6, 5, (_Float64) -1*M_PI/2));
    path.push_back(setPose(10, 10, 3, 0));
    path.push_back(setPose(1, 1, 1, (_Float64) M_PI));

    ServiceClient set_mode = node.serviceClient<SetMode>(set_mode_service);
    ServiceClient arm = node.serviceClient<CommandBool>(arm_service);
    ServiceClient takeoff = node.serviceClient<CommandTOL>(takeoff_service);
    ServiceClient land = node.serviceClient<CommandTOL>(land_service);

    Subscriber state = node.subscribe(state_subscription, queue_size, getState);
    Subscriber position = node.subscribe(position_subscription, queue_size, updatePosition);

    Publisher target_position = node.advertise<PoseStamped>(position_publisher, queue_size);
    Publisher fruit_count = node.advertise<Int32>(topic_name, queue_size);

    Rate loop_rate(rate);

    ROS_INFO("Connecting to Drone...");
    while(ok() && !isConnected) {
        spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Drone Connected!");

    ROS_INFO("Setting Mode to GUIDED...");
    mode_command.request.custom_mode = mode;
    set_mode.call(mode_command);
    while(ok() && !mode_command.response.mode_sent && !isGuided) {
        spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Mode set to GUIDED!");

    ROS_INFO("Arming the Drone...");
    arm_command.request.value = true;
    arm.call(arm_command);
    while(ok() && arm_command.response.success && !isArmed) {
        spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Armed the Drone!");

    /*
    ROS_INFO("Initiating Takeoff...");
    takeoff_command.request.altitude = altitude;
    takeoff.call(takeoff_command);
    while( abs(altitude-current_position.pose.position.z) > tolerance ) {
        spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Takeoff Successful! Altitude = %f", altitude);
    */

    goToPose(startPosition, target_position, tolerance, loop_rate);
    for(vector<PoseStamped>::iterator vertex = path.begin(); vertex != path.end(); vertex++)
        goToPose(*vertex, target_position, tolerance, loop_rate);

    /*
    ROS_INFO("Going to Initial Position (Position of Takeoff and now Landing)...");
    goToPose(startPosition, target_position, tolerance, loop_rate);
    goToPose(initialPosition, target_position, tolerance, loop_rate);
    ROS_INFO("Reached Initial Position");
    */

    /*
    ROS_INFO("Landing...");
    land_command.request.altitude = 0;
    land.call(land_command);
    while( current_position.pose.position.z > tolerance ) {
        spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Landing Successful!");
    */
    Int32 fruit_data;
    fruit_data.data = 42;
    fruit_count.publish(fruit_data);

    return 0;
}
