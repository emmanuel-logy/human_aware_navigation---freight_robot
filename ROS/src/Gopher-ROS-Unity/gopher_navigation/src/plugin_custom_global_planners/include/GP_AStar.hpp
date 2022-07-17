/**
 * This module contains the class handling the PID controller computation
 */

#ifndef GP_ASTAR_HPP
#define GP_ASTAR_HPP

// my headers
#include "Node.hpp"

// ROS libraries for implementing global planners
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


namespace custom_global_planners
{
	class GP_AStar : public nav_core::BaseGlobalPlanner
	{

	public:

		GP_AStar();
		~GP_AStar();
		GP_AStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		bool makePlan(const geometry_msgs::PoseStamped& start,
					  const geometry_msgs::PoseStamped& goal,
					  std::vector<geometry_msgs::PoseStamped>& plan);


	private:

		bool m_initialized;

		costmap_2d::Costmap2DROS* m_costmap_ros;

		costmap_2d::Costmap2D* m_costmap;

		unsigned int m_total_rows;

		unsigned int m_total_cols;

		MatrixXNode m_graph;			// input map

		geometry_msgs::PoseStamped m_prev_start;

		std::vector<geometry_msgs::PoseStamped>* m_plan;

		geometry_msgs::PoseStamped m_prev_goal;

		bool m_found;
	};

}    	// namespace scara

#endif	// #ifndef GP_ASTAR_HPP
