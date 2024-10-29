// Author: Claude Sammut

#include "pl_interface/pl_interface.hpp"

#include <memory>


using namespace std::chrono_literals;

static bool pl_near_start(term, term *);
static term pl_scan(term, term *);
static bool pl_drive(term, term *);
PlInterface *current_node;

PlInterface::PlInterface() : Node("pl_interface_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

	robot_pose_ = 0.0;
	near_start = false;

	/************************************************************
	** Initialise iProlog
	************************************************************/
	pl_init();
	new_pred(pl_near_start, (char *) "near_start");
	new_subr(pl_scan, (char *) "scan");
	new_pred(pl_drive, (char *) "drive");
	ONE((char *) "[]", (char *) "consult('wall_follower.pro')");
	RCLCPP_INFO(this->get_logger(), "Prolog wall follower loaded");

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&PlInterface::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&PlInterface::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&PlInterface::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "iProlog interface node has been initialised");
}

PlInterface::~PlInterface()
{
	RCLCPP_INFO(this->get_logger(), "iProlog interface node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.2

void PlInterface::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

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
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start!!\n");
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 10

void PlInterface::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

void PlInterface::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

void PlInterface::update_callback()
{
	current_node = this;
	pl_query(intern((char *) "wall_follower"), _nil, 1);
}


/********************************************************************************
** Prolog hooks
********************************************************************************/

bool pl_near_start(term goal, term *frame)
{
	return current_node -> near_start;
}

term pl_scan(term goal, term *frame)
{
	term x = check_arg(1, goal, frame, INT, EVAL);
	int angle = IVAL(x);

	return new_real(current_node -> scan_data_[angle]);
}

bool pl_drive(term goal, term *frame)
{
	term lv = check_arg(1, goal, frame, REAL, EVAL);
	term av = check_arg(2, goal, frame, REAL, EVAL);

	current_node -> update_cmd_vel(RVAL(lv), RVAL(av));
	return true;
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PlInterface>());
	rclcpp::shutdown();

	return 0;
}
