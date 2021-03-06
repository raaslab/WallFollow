#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include "wall_follow/Lines.h"
#include <cmath>
#include "tf/transform_datatypes.h"

enum Mode {follow_wall, recover_altitude};
Mode current_mode = recover_altitude;
const float desired_wall_dist = 5.0;
const float desired_altitude = 5.0;
const float altitude_entry_tolerance = 0.2;
const float altitude_exit_tolerance = 1;
const float confidence_threshold = 10;
const int laser_rf_offset = 180;				// lidar (Sweep) leads robot x axis (east) by 90 degrees in simulation and 180 degrees on the actual quad
						// verify and make sure that both are anticlockwise

bool new_data = 0;

float hold_error;
float hold_velocity;
float prev_hold_errors[5];
const float Kp = 1;
const float Kd = 0.15;
const float Ki = 0;
float max_velocity = 0.5;
float nominal_velocity = 0.5;

geometry_msgs::PoseStamped local_pose;
mavros_msgs::State current_state;
wall_follow::Lines hough_lines;

using namespace std;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	local_pose = *msg;
}

void lines_cb(const wall_follow::Lines::ConstPtr& lines){
	//ROS_INFO("first line: dist:%2.4f, angle:%d, confidence:%d",lines->dist[0], lines->angle[0], lines->confidence[0]);
	hough_lines = *lines;
	new_data = 1;
}



mavros_msgs::PositionTarget computeTargetVel(){
	mavros_msgs::PositionTarget target_vel;
	target_vel.header.stamp = ros::Time::now();
	target_vel.header.frame_id = "local_frame";
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target_vel.type_mask = 0b111111000111;
	target_vel.yaw_rate = 0;
	int orientation = 	(int)(tf::getYaw(local_pose.pose.orientation) * 180 / M_PI);
	//if(orientation < 0)
		//orientation = 360 + orientation;
	//ROS_INFO("yaw = %d", orientation);

	//int hough_angle_rf = hough_lines.angle[0];					//lf = local frame, not laser frame!!!

	ROS_INFO("altitude: %f", local_pose.pose.position.z);
	ROS_INFO("confidence: %d", hough_lines.confidence[0]);

 	target_vel.yaw_rate = 0;
	target_vel.velocity.x = 0;
	target_vel.velocity.y = 0;
	target_vel.velocity.z = 0;
	

	return target_vel;
}


int main(int argc, char **argv){
//_____________________________________________________________________________________________________#InitializeNodePubSubAndRate

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

	ros::Subscriber lines_sub = nh.subscribe<wall_follow::Lines>("ho/li",10,lines_cb);
	ros::Publisher velocity_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	//ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    ros::Rate rate(20.0);		//the setpoint publishing rate MUST be faster than 2Hz
										
    while(ros::ok() && current_state.connected){	// wait for FCU connection	
        ros::spinOnce();
        rate.sleep();
    }


//___________________________________________________________________________________________________________________#dummysetpoints

													// body: yaw is relative
													// offset: translation is relative
													// local frame: whereever we 

	mavros_msgs::PositionTarget target_vel;
	target_vel.header.stamp = ros::Time::now();
	target_vel.header.frame_id = "local_frame";
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target_vel.type_mask = 0b111111000111;
	target_vel.yaw_rate = 0;
	target_vel.velocity.x = 0;
	target_vel.velocity.y = 0;
	target_vel.velocity.z = 0;


	for(int i = 100; ros::ok() && i > 0; --i){
		velocity_pub.publish(target_vel);
		rate.sleep();
	}




//__________________________________________________________________________________________________________________ #mainloop

    while(ros::ok()){

        ros::spinOnce();
	velocity_pub.publish(computeTargetVel());
	ROS_INFO("main loop: publishing velocity at 20 Hz");
        rate.sleep();
    }

    return 0;
}
