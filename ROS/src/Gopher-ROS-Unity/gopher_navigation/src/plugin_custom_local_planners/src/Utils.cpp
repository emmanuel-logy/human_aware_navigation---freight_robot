/**
 * This module contains Dijkstra, DFS, Djikstras, A* algorithms
 */

#include "Utils.hpp"
#include <tf/tf.h>
#include <algorithm>

namespace custom_global_planners
{

	void Utils::initialize_graph(const costmap_2d::Costmap2D* costmap,
								 MatrixXNode& graph)
	{
		int rows = costmap->getSizeInCellsX();
		int cols = costmap->getSizeInCellsY();

		bool is_obs = false;
		// Fill the graph matrix with Nodes corresponding to grid information
		for (int i=0; i<rows; ++i)
		{
			for (int j=0; j<cols; ++j)
			{
		        is_obs = (static_cast<int>(costmap->getCost(i, j)) > 1) ? true : false;

				graph(i,j) = make_shared<Node>(i, j, is_obs);
			}
		}
	}


	void Utils::find_valid_neighbors(const MatrixXNode& graph,
									 const shared_ptr<Node> current_node,
								     vector<pair<shared_ptr<Node>,int>>& neighbors_w_list)
	{
		neighbors_w_list.clear();

		int total_rows = graph.rows();
		int total_cols = graph.cols();

		int row = current_node->row;
		int col = current_node->col;

		// Right neighbor
		if ( (col+1)< total_cols && !graph(row, col+1)->obs)
		{
			auto R = graph(row, col+1);
			neighbors_w_list.push_back(make_pair(R, current_node->edge_w[0]));
		}

		// Down neighbor
		if ( (row+1)< total_rows && !graph(row+1, col)->obs)
		{
			auto D = graph(row+1, col);
			neighbors_w_list.push_back(make_pair(D, current_node->edge_w[1]));
		}

		// Left neighbor
		if ( (col-1)> 0 && !graph(row, col-1)->obs)
		{
			auto L = graph(row, col-1);
			neighbors_w_list.push_back(make_pair(L, current_node->edge_w[2]));
		}

		// Top neighbor
		if ( (row-1)> 0 && !graph(row-1, col)->obs)
		{
			auto T = graph(row-1, col);
			neighbors_w_list.push_back(make_pair(T, current_node->edge_w[3]));
		}
	}


	bool Utils::generate_path(const MatrixXNode& graph,
							const Eigen::Vector2i& start,
						    const Eigen::Vector2i& goal,
							const costmap_2d::Costmap2D* costmap,
							std::vector<geometry_msgs::PoseStamped>& plan)
	{
		plan.clear();

		double goal_wx = 0;
		double goal_wy = 0;
		costmap->mapToWorld(goal(0), goal(1), goal_wx, goal_wy);

		auto current_node = graph(goal(0), goal(1));

		// If parent doesn't exist for goal node, then path was not found
		if (current_node->parent == nullptr)
		{
			plan.clear();
			return false;
		}
		else
		{
			// Starting from goal node, move to start node using parent info to find the path
			geometry_msgs::PoseStamped tmp;
			tmp.pose.position.z = 0.0;
			tmp.header.frame_id = "map";
			while (current_node != graph(start(0), start(1)))
			{
				costmap->mapToWorld(current_node->row, current_node->col,
									tmp.pose.position.x, tmp.pose.position.y);

				double angle = atan2((double)(goal_wy - tmp.pose.position.y),
									 (double)(goal_wx - tmp.pose.position.x));

		        tmp.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
				tmp.header.stamp = ros::Time::now();

				plan.push_back(tmp);

				current_node = graph(current_node->parent->row, current_node->parent->col);
			}

			// push start node also
			costmap->mapToWorld(current_node->row, current_node->col,
								tmp.pose.position.x, tmp.pose.position.y);
			plan.push_back(tmp);

			// get path from start to goal
			reverse(plan.begin(), plan.end());
			return true;
		}
	}


}	// namespace motion_planning
