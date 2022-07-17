/**
 * This module contains the class handling the PID controller computation
 */

// my headers
#include "LP_MPC.hpp"
#include "Utils.hpp"
// ros headers
#include <pluginlib/class_list_macros.h>
// C++ STL headers
#include <iostream>
using namespace std;

PLUGINLIB_EXPORT_CLASS(custom_local_planners::LP_MPC, nav_core::BaseLocalPlanner);

namespace custom_local_planners
{
	LP_MPC::LP_MPC() : m_costmap_ros(NULL), m_tf(NULL), m_initialized(false)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
	}


	LP_MPC::LP_MPC(std::string name, tf2_ros::Buffer* tf_, costmap_2d::Costmap2DROS* costmap_ros) :  m_costmap_ros(NULL),
																										   m_tf(NULL),
																										   m_initialized(false)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
		initialize(name, tf_, costmap_ros);
	}


	LP_MPC::~LP_MPC()
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
	}


	void LP_MPC::initialize(std::string name, tf2_ros::Buffer* tf_, costmap_2d::Costmap2DROS* costmap_ros)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;

		if(!m_initialized)
		{
			m_tf = tf_;
			m_costmap_ros = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
			m_costmap = m_costmap_ros->getCostmap(); //get the m_costmap from costmap_ros_
			m_orig_global_plan = nullptr;
			m_initialized = true;
		}
	}


	bool LP_MPC::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;

		if(!m_initialized)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		if ( m_orig_global_plan &&
			 orig_global_plan.back() != m_orig_global_plan->back() )
		{
			if (m_orig_global_plan)
				delete m_orig_global_plan;

			m_orig_global_plan = new std::vector<geometry_msgs::PoseStamped>;
			*m_orig_global_plan = orig_global_plan;
		}

		return true;
	}


	bool LP_MPC::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;

		if(!m_initialized)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		return false;
	}


	bool LP_MPC::isGoalReached()
	{
		cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;

		if(!m_initialized)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		m_costmap_ros->getRobotPose(m_robot_pose);

		if ( m_orig_global_plan &&
			 m_robot_pose == m_orig_global_plan->back() )
		{
			cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
			return true;
		}
		else
		{
			cout << __FILE__ << ": " << __func__ << ": " << __LINE__ << endl;
			return false;
		}
	}

} // namespace custom_local_planner
