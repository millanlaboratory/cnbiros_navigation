#ifndef CNBIROS_NAVIGATION_FORCEFIELD_HPP
#define CNBIROS_NAVIGATION_FORCEFIELD_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_fusion/FusionGrid.hpp"

#include "cnbiros_navigation/Flags.hpp"
#include "cnbiros_navigation/TrigTools.hpp"
#include "cnbiros_navigation/SetStrengthSrv.h"
#include "cnbiros_navigation/SetDecaySrv.h"

#include <limits>
#include <cmath>

namespace cnbiros {
	namespace navigation {

class ForceField : public cnbiros::core::NodeInterface {

	public:
		ForceField(ros::NodeHandle* node, std::string name = "forcefield");
		virtual ~ForceField(void);

		void SetTargetLayer(const std::string& layer, unsigned int type = ForceField::ForBoth);
		void SetStrength(float value, unsigned int type);
		void SetDecay(float value, unsigned int type);
		void SetRobotSize(float size, float sector);


	protected:
		void on_received_attractors(const grid_map_msgs::GridMap::ConstPtr& msg);
		void on_received_repellors(const grid_map_msgs::GridMap::ConstPtr& msg);
		
		bool on_set_strength(cnbiros_navigation::SetStrengthSrv::Request& req,
							 cnbiros_navigation::SetStrengthSrv::Response& res);
		bool on_set_decay(cnbiros_navigation::SetDecaySrv::Request& req,
						  cnbiros_navigation::SetDecaySrv::Response& res);
		void convert_grid_to_sector(grid_map::GridMap& grid, std::string layer,  
										std::vector<float>& sectors);
		float compute_angular_velocity(std::vector<float>& sectors, 
											float beta1, float beta2);
		float compute_velocity_linear(std::vector<float>& sectors, float maxvel, 
								float safezone, float decay, float audacity);

		virtual void onRunning(void);

	
		//float compute_angle(float x, float y);
		//float compute_distance(float x, float y);

	public:
		const static unsigned int ForAttractors = 1;
		const static unsigned int ForRepellors  = 2;
		const static unsigned int ForBoth       = 3;

	private:
		ros::Subscriber		rossub_attractors_;
		ros::Subscriber		rossub_repellors_;
		ros::Publisher		rospub_cmdvel_;
		ros::ServiceServer 	rossrv_set_strength_;
		ros::ServiceServer 	rossrv_set_decay_;



		std::string 		a_layer_;
		float 				a_beta1_;
		float 				a_beta2_;
		grid_map::GridMap	a_grid_;
		
		std::string 		r_layer_;
		float 				r_beta1_;
		float 				r_beta2_;
		grid_map::GridMap	r_grid_;

		float 				robot_size_;
		float 				robot_sector_;

		float 				angular_velocity_;
		float 				linear_velocity_;

		//unsigned int		n_sects_;
		//where are vectors actually initialized??
		std::vector<float> 	r_sectors_; 
		//(n_sects,std::numeric_limits<float>::infinity());
		std::vector<float> 	a_sectors_; 
		//(n_sects,std::numeric_limits<float>::infinity());



};


	}
}

#endif
