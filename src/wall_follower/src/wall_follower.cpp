// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower


#include "wall_follower/wall_follower.hpp"
#include <memory>
#include <fstream>
using namespace std::chrono_literals;


WallFollower::WallFollower()
: Node("wall_follower_node")
{
   /************************************************************
   ** Initialise variables
   ************************************************************/
   for (int i = 0; i < 12; i++)
       scan_data_[i] = 0.0;

   robot_pose_ = 0.0;
   near_start = false;
   first_bool = false;

   // Turning Variables
   turning_left = false;
   turning_right = false;
   moving_forward_after_turning_left = false;
   
   // Course Correction Variables
   correct_left = false;
   correct_right = false;

   /************************************************************
   ** Initialise ROS publishers and subscribers
   ************************************************************/
   auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

   // Initialise publishers
   cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
   nav_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_complete", qos);

   // Initialise subscribers
   scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
       "scan", \
       rclcpp::SensorDataQoS(), \
       std::bind(
           &WallFollower::scan_callback, \
           this, \
           std::placeholders::_1));
   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       "odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));


   /************************************************************
   ** Initialise ROS timers
   ************************************************************/
   update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));


   RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}


WallFollower::~WallFollower()
{
   RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}


/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE 0.2

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool within_start_range = true;

	tf2::Quaternion q(
    	msg->pose.pose.orientation.x,
    	msg->pose.pose.orientation.y,
    	msg->pose.pose.orientation.z,
    	msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);


	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	if (first){
		start_x = current_x;
		start_y = current_y;
		first = false;
		// TODO check path
		std::ofstream file("/home/comp3431/colcon_ws/src/wall_follower/scripts/positions.csv", std::ios::trunc);
		// // safety check
		if (!file) {
		    std::string print = "File does not exist\n";
			RCLCPP_INFO(this->get_logger(), print);
		}
		
		// do these values need any formatting or typecasting?
		file << start_x << "," << start_y << "," << yaw << "\n";
		file.close();
		fprintf(stderr, "This is first!!\n");
	}
	else if (within_start_range)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE){
			fprintf(stderr, "out of START RANGE\n");
			within_start_range = false;
		}
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE){
		fprintf(stderr, "Near start!!\n");
		near_start = true;
	}
	// fprintf(stderr, "within range: %s\n", within_start_range ? "true" : "false");
    // fprintf(stderr, "near_start: %s\n", near_start ? "true" : "false");

}


#define BEAM_WIDTH 10

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
   uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) == 0)
			continue;
		else if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) == 0)
			continue;
		else if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	int beam_width_var = 10;
	for (int i = 1; i < 12; i++)
	{	
		// if FRONT_RIGHT, then beam width is 15
		if (i == 11){
			beam_width_var = 15;
		} else {
			beam_width_var = 10;
		}

		closest = msg->range_max;
		for (int angle = scan_angle[i]-beam_width_var; angle < scan_angle[i]+beam_width_var; angle++)
			if (msg->ranges.at(angle) == 0)
				continue;
			else if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}


void WallFollower::update_cmd_vel(double linear, double angular)
{
   geometry_msgs::msg::Twist cmd_vel;
   cmd_vel.linear.x = linear;
   cmd_vel.angular.z = angular;


   cmd_vel_pub_->publish(cmd_vel);
}

void WallFollower::update_nav()
{	
	if (first_bool == false){
		first_bool = true;
		std_msgs::msg::Bool nav_complete_msg;
		nav_complete_msg.data = true;
		
		nav_complete_pub_->publish(nav_complete_msg);
	}
}


/********************************************************************************
** Update functions
********************************************************************************/

void WallFollower::update_callback()
{

	////////////////////////
	//// IGNORE ALL 0s /////
	////////////////////////

	bool allzero = true;
	for (int i = 0; i < 12; i++) {
		if (scan_data_[i] != 0.0) {
			allzero = false;
			break;
		}
	}
	if (allzero) {
		RCLCPP_WARN(this->get_logger(), "All scan data values are zero. Returning...");
		return;
	}

	///////////////////////////////
	//// CURRENTLY TURNING CODE ///
	///////////////////////////////
	
	if (moving_forward_after_turning_left) {
		// checks if there is a wall on the left
		if (scan_data_[LEFT] > 0.5) { 	
			update_cmd_vel(0.1, 0.0);
			std::string print = "After turning LEFT, should be moving FORWARD\n";
			RCLCPP_INFO(this->get_logger(), print);
		} else {
			update_cmd_vel(0.0, 0.0);
			moving_forward_after_turning_left = false;
			std::string print = "Completed moving FORWARD after LEFT turn\n";
			RCLCPP_INFO(this->get_logger(), print);
		}

	} else if (turning_left) {
		// Currently turning left
		double yaw_diff = robot_pose_ - prev_yaw;
		yaw_diff = normalize_angle(yaw_diff);

		if (fabs(yaw_diff) >= 90*DEG2RAD) {
			update_cmd_vel(0,0); // Stop turning
			turning_left = false;
			moving_forward_after_turning_left = true;
			RCLCPP_INFO(this->get_logger(), "Completed 90-degree LEFT TURN");
		} else {
			update_cmd_vel(0.03,0.5);
			RCLCPP_INFO(this->get_logger(), "Performing 90-degree LEFT TURN");
		}

		RCLCPP_INFO(this->get_logger(), "\n Here is yaw_diff");
		RCLCPP_INFO(this->get_logger(), std::to_string(yaw_diff));
		RCLCPP_INFO(this->get_logger(), "\n\n");
		
	} else if (turning_right) {
		// Currently turning right
		double yaw_diff = robot_pose_ - prev_yaw;
		yaw_diff = normalize_angle(yaw_diff);

		if (fabs(yaw_diff) >= 90*DEG2RAD) {
			update_cmd_vel(0,0); // Stop turning
			turning_right = false;
			RCLCPP_INFO(this->get_logger(), "Completed 90-degree RIGHT TURN");
		} else {
			update_cmd_vel(0,-0.5);
			RCLCPP_INFO(this->get_logger(), "Performing 90-degree RIGHT TURN");
		}
	
	} else if (correct_right) {
		// Currently correcting right
		double yaw_diff = robot_pose_ - prev_yaw;
		yaw_diff = normalize_angle(yaw_diff);

		if (fabs(yaw_diff) >= 3*DEG2RAD || scan_data_[FRONT_LEFT] >= 0.5) {
			update_cmd_vel(0,0); // Stop turning
			correct_right = false;
			RCLCPP_INFO(this->get_logger(), "Completed 5-degree RIGHT CORRECTION TURN");
		} else {
			update_cmd_vel(0.1,-0.14);
			RCLCPP_INFO(this->get_logger(), "Performing 5-degree RIGHT CORRECTION TURN");
		}

	} else if (correct_left) {
		// Currently correcting left
		double yaw_diff = robot_pose_ - prev_yaw;
		yaw_diff = normalize_angle(yaw_diff);

		if (fabs(yaw_diff) >= 3*DEG2RAD || scan_data_[FRONT_LEFT] <= 0.5) {
			update_cmd_vel(0,0); // Stop turning
			correct_left = false;
			RCLCPP_INFO(this->get_logger(), "Completed 5-degree LEFT CORRECTION TURN");
		} else {
			update_cmd_vel(0.1, 0.14);
			RCLCPP_INFO(this->get_logger(), "Performing 5-degree LEFT CORRECTION TURN");
		}

	} else {

		/////////////////////////////
		//// DECISION MAKING CODE ///
		/////////////////////////////

		if (near_start) {
			update_cmd_vel(0.0, 0.2);
			
			std::string print1 = "Sending Bool";
			RCLCPP_INFO(this->get_logger(), print1);
			
			update_nav();

			print1 = "NEAR_START (shouldn't be doing anything)\n";
			RCLCPP_INFO(this->get_logger(), print1);
		
		} else if (scan_data_[LEFT] > 0.65) {
			// does the FRONT_LEFT clause cause it to over turn sometimes, as it keeps going forward
			// till it finds that high value?
			prev_yaw = robot_pose_;
			turning_left = true;
			std::string print2 = "LEFT > 0.8 (should be JUST turning LEFT)\n";
			RCLCPP_INFO(this->get_logger(), print2);

		} else if ((scan_data_[FRONT] < 0.4) || (scan_data_[FRONT_RIGHT] < 0.45)){
			prev_yaw = robot_pose_;
			turning_right = true;
			std::string print4 = "FRONT < 0.4 || FRONT_RIGHT < 0.5 (should be JUST turning RIGHT)\n";
			RCLCPP_INFO(this->get_logger(), print4);

		///////////////////////////////
		//// COURSE CORRECTION CODE ///
		///////////////////////////////
		
		} else if (scan_data_[FRONT_LEFT] < 0.45) {
			prev_yaw = robot_pose_;
			correct_right = true;
			
			std::string print2 = "FRONT_LEFT< 0.44 (should be correcting RIGHT)\n";
			RCLCPP_INFO(this->get_logger(), print2);
		}
		else if (scan_data_[FRONT_LEFT] > 0.55 && scan_data_[FRONT_LEFT] < 0.75){
			prev_yaw = robot_pose_;
			correct_left = true;
			
			std::string print4 = "FRONT_LEFT > 0.55 (should be correcting LEFT)\n";
			RCLCPP_INFO(this->get_logger(), print4);
		
		} else {
			update_cmd_vel(0.15, 0.0);

			std::string print7 = "Here (should be JUST moving forward)\n";
			RCLCPP_INFO(this->get_logger(), print7);
		}
	}

	print_scan_data();
}


/*******************************************************************************
** Helper Functions
*******************************************************************************/

int WallFollower::get_min_distance_and_direction()
{
    double curr_min_distance = scan_data_[0];
    int curr_min_direction = 0;

    for (int i = 0; i < 12; i++) {
        if (scan_data_[i] < curr_min_distance) {
            curr_min_distance = scan_data_[i];
            curr_min_direction = i;
        }
    }

    return curr_min_direction;
}


double WallFollower::normalize_angle(double angle) {
    if (angle > M_PI) angle -= 2*M_PI;
    else if (angle < -M_PI) angle += 2*M_PI;
    return angle;
}


void WallFollower::print_scan_data() {
    std::string angle_i = "scan_angle: ";
    std::string scan_data_i = "scan_data: ";
    for (int i = 0; i < 12; i++) {
        int angle = (i) * 30;
        angle_i += std::to_string(angle) + " ";
        scan_data_i += std::to_string(scan_data_[i]) + " ";
    }
	angle_i += "\n";

	RCLCPP_INFO(this->get_logger(), angle_i);
	RCLCPP_INFO(this->get_logger(), scan_data_i);
	RCLCPP_INFO(this->get_logger(), "\n\n");
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<WallFollower>());
   rclcpp::shutdown();


   return 0;
}