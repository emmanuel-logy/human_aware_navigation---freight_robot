/**
 * This module contains the class handling the PID controller computation
 */

#ifndef LP_MPC_HPP
#define LP_MPC_HPP

// my headers

// ROS libraries for implementing global planners
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_listener.h>


namespace custom_local_planners
{
	class LP_MPC : public nav_core::BaseLocalPlanner
	{

	public:

		LP_MPC();

		LP_MPC(std::string name, tf2_ros::Buffer* tf_, costmap_2d::Costmap2DROS* costmap_ros);

		~LP_MPC();

		void initialize(std::string name, tf2_ros::Buffer* tf_, costmap_2d::Costmap2DROS* costmap_ros);

		bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

		bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

		bool isGoalReached();

	private:

		bool m_initialized;

		costmap_2d::Costmap2DROS* m_costmap_ros;

		costmap_2d::Costmap2D* m_costmap;

		tf2_ros::Buffer* m_tf;

		std::vector<geometry_msgs::PoseStamped>* m_orig_global_plan;

		geometry_msgs::PoseStamped m_robot_pose;


	};

}    	// namespace custom_local_planners

#endif	// #ifndef LP_MPC_HPP
