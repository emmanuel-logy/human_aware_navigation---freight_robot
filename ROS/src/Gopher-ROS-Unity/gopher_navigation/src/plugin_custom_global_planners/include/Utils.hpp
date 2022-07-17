/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#ifndef HW1_BASIC_SEARCH_UTILS_HPP
#define HW1_BASIC_SEARCH_UTILS_HPP

#include "Node.hpp"
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
using namespace std;


namespace custom_global_planners
{
	class Utils
	{
	public:

		static void initialize_graph(const costmap_2d::Costmap2D* costmap,
									 MatrixXNode& graph);


		static void find_valid_neighbors(const MatrixXNode& graph,
										 const shared_ptr<Node> current_node,
										 vector<pair<shared_ptr<Node>,int>>& neighbors_w_list);


		static bool generate_path(const MatrixXNode& graph,
								const Eigen::Vector2i& start,
								const Eigen::Vector2i& goal,
								const costmap_2d::Costmap2D* costmap,
								std::vector<geometry_msgs::PoseStamped>& plan);

	private:

	};

}  // namespace motion_planning

#endif	// #ifndef HW1_BASIC_SEARCH_UTILS_HPP
