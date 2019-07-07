#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>

bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node,
	int frame, int command, bool isCurrent, bool autoCont,
	float param1, float param2, float param3, float param4,
	float lat, float lon, float alt );

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::ServiceClient pushClient;
    mavros_msgs::WaypointPush wayPusher;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ROS_INFO("Initializing ...");
    // WAIT FOR FCU CONNECTION
    ROS_INFO("Waiting for FCU connection ...");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connection estabilished");
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //SEND A FEW WAYPOINTS BEFORE STARTING
    ROS_INFO("Sending setpoints before starting");
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

		// LOAD WAYPOINT'S LIST
    waypointPusher(wayPusher, pushClient, nh, mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT, mavros_msgs::CommandCode::NAV_TAKEOFF, true, true, 0, 0, 0, 0, 40.6340638, -8.6605780, 4);

		waypointPusher(wayPusher, pushClient, nh, mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT, mavros_msgs::CommandCode::NAV_WAYPOINT, true, true, 15, 0, 0, 0, 40.6339142, -8.6606963, 10);

		waypointPusher(wayPusher, pushClient, nh, mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT, mavros_msgs::CommandCode::NAV_LOITER_TIME, true, true, 15, 0, 1, 0, 40.633884, -8.660726, 15);

		waypointPusher(wayPusher, pushClient, nh, mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT, mavros_msgs::CommandCode::NAV_LOITER_TURNS, true, true, 5, 0, 1, 0, 40.6339142, -8.6606963, 10);

		waypointPusher(wayPusher, pushClient, nh, mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT, mavros_msgs::CommandCode::NAV_LAND, true, true, 0, 0, 0, 0, 40.6340783, -8.6604549, 5);

    // CHANGE TO MISSION MODE
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.MISSION";
    set_mode_client.call(offb_set_mode);

    // ARM DRONE
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client.call(arm_cmd);

    // WAIT FOR MISSION TO COMPLETE
    while(ros::ok())
	  {
		ros::spinOnce();
		rate.sleep();
	  }

    ROS_INFO("Landing");
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);

    return 0;
}

bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node,
	int frame, int command, bool isCurrent, bool autoCont,
	float param1, float param2, float param3, float param4,
	float lat, float lon, float alt )
{
	client = node.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

	mavros_msgs::Waypoint nextWaypoint;

	nextWaypoint.frame = frame;
	nextWaypoint.command = command;
	nextWaypoint.is_current = isCurrent;
	nextWaypoint.autocontinue = autoCont;
	nextWaypoint.param1 = param1;
	nextWaypoint.param2 = param2;
	nextWaypoint.param3 = param3;
	nextWaypoint.param4 = param4;
	nextWaypoint.x_lat = lat;
	nextWaypoint.y_long = lon;
	nextWaypoint.z_alt = alt;

	pusher.request.waypoints.push_back(nextWaypoint);

	if( client.call( pusher) )
		ROS_INFO_STREAM("PUSHED WAYPOINT");
	else
		ROS_INFO_STREAM("PUSH FAILED");

	return client.call(pusher);
}
