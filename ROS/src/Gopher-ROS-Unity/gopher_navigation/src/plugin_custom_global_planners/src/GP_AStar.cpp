/**
 * This module contains the class handling the PID controller computation
 */

// my headers
#include "GP_AStar.hpp"
#include "Utils.hpp"
// ros headers
#include <pluginlib/class_list_macros.h>
// C++ STL headers
#include <iostream>
using namespace std;
PLUGINLIB_EXPORT_CLASS(custom_global_planners::GP_AStar, nav_core::BaseGlobalPlanner);

namespace custom_global_planners
{

	// To order the priority queue in the ascedning order
	class cmp
	{
	public:
		bool operator() (const shared_ptr<Node> n1, const shared_ptr<Node> n2)
		{
			return (n1->cost  >  n2->cost);
		}
	};


	GP_AStar::GP_AStar() : 	m_plan (nullptr),
							m_found (false)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
	}


	GP_AStar::~GP_AStar()
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
		if (m_plan)
			delete m_plan;
	}


	GP_AStar::GP_AStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
	}

	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void GP_AStar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;

		if(!m_initialized)
		{
			m_costmap_ros = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
			m_costmap = m_costmap_ros->getCostmap(); //get the m_costmap from costmap_ros_

			// initialize other planner parameters
//			ros::NodeHandle private_nh("~/" + name);
//			private_nh.param("step_size", step_size_, m_costmap->getResolution());
//			private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
//			world_model_ = new base_local_planner::CostmapModel(*m_costmap);



			// [1] Converting the input grid into graph of Nodes and initializing each node
			m_total_rows = m_costmap->getSizeInCellsX();
			m_total_cols = m_costmap->getSizeInCellsY();
			m_graph = MatrixXNode(m_total_rows, m_total_cols);		// empty graph matrix is ready
			Utils::initialize_graph(m_costmap, m_graph);

			m_initialized = true;
		}
		else
		{
			ROS_WARN("This planner has already been initialized... doing nothing");
		}
	}

	bool GP_AStar::makePlan(const geometry_msgs::PoseStamped& start,
							const geometry_msgs::PoseStamped& goal,
							std::vector<geometry_msgs::PoseStamped>& plan)
	{
		cout << "=======================================================" << endl;
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
		cout << "Current Start: " << start << endl;
		geometry_msgs::PoseStamped pose;
		m_costmap_ros->getRobotPose(pose);
		cout << "Current Pose : " << pose << endl;
		cout << "=======================================================" << endl;


		if ( (goal == m_prev_goal) && m_found )
		{
			plan = *m_plan;
			cout << "Already found path: " << m_plan->size() << endl;
			return true;
		}

		// [0] Check if m_graph is initialized
		if(!m_initialized)
		{
			ROS_WARN("This planner has not been initialized..!");
			return false;
		}

		// [0] Bookkeeping
		m_found = false;
		m_prev_goal = goal;

		// [1] Clearing garbage values in output variables
		plan.clear();

		// [2] Start & Goal node
		unsigned int start_mx = 0;
		unsigned int start_my = 0;
		m_costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
		auto start_node = m_graph(start_mx, start_my);

		unsigned int goal_mx = 0;
		unsigned int goal_my = 0;
		m_costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);
		auto goal_node = m_graph(goal_mx, goal_my);


		// [3] Queuing the nodes to visit in the loop
		priority_queue <shared_ptr<Node>,
						vector<shared_ptr<Node>>,
						cmp> Q;
		Q.push(start_node);


		// [4] Update each nodes' parents and distance from start using Dijkstra
		auto current_node = Q.top();
		vector<pair<shared_ptr<Node>,int>> neighbors_w;
		int row=0, col=0;

		while ( (!Q.empty()) && (current_node != goal_node) )
		{
			current_node = Q.top();
			Q.pop();

			row = current_node->row;
			col = current_node->col;

			neighbors_w.clear();
			Utils::find_valid_neighbors(m_graph, current_node, neighbors_w);

			for (auto& node_w : neighbors_w)
			{
				auto node = node_w.first;
				auto w = node_w.second;

				auto d = current_node->cost + w;
				if ( !node->visited && d < node->cost )
				{
					node->cost 	 	= d;
					node->parent 	= current_node;
					node->visited	= true;
					Q.push(node);
				}
			}

			current_node->visited = true;
		}


		// [5] Using updated graph's info, find the path from start to goal
		const Eigen::Vector2i start2d {start_mx, start_my};
		const Eigen::Vector2i goal2d {goal_mx, goal_my};

		bool res = Utils::generate_path(m_graph, start2d, goal2d, m_costmap, plan);

		if (res)
		{
			cout << "Found: " << plan.size() << endl;
			m_found = true;

			if (m_plan)
				delete m_plan;
			m_plan = new std::vector<geometry_msgs::PoseStamped>;
			*m_plan = plan;
		}
		else
		{
			cout << "Not Found: " << plan.size() << endl;
			m_found = false;
		}
		return res;
	}


} // namespace scara
