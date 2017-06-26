#ifndef CNBIROS_NAVIGATION_FORCEFIELD_HPP
#define CNBIROS_NAVIGATION_FORCEFIELD_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_navigation/Flags.hpp"
#include "cnbiros_navigation/FusionGrid.hpp"

namespace cnbiros {
	namespace navigation {

class ForceField : public cnbiros::core::NodeInterface {

	public:
		ForceField(ros::NodeHandle* node, std::string name = "forcefield");
		virtual ~ForceField(void);

		void SetTargetLayer(const std::string& layer, unsigned int type = ForceField::ForBoth);
		void SetStrength(float value, unsigned int type);
		void SetDecay(float value, unsigned int type);
		void SetObstruction(float value, unsigned int type);


	protected:
		void on_received_attractors(const grid_map_msgs::GridMap::ConstPtr& msg);
		void on_received_repellors(const grid_map_msgs::GridMap::ConstPtr& msg);
		void on_received_odometry(const nav_msgs::Odometry::ConstPtr& msg);
		
		float compute_velocity_angular(FusionGrid& grid, std::string layer, 
									   float decay, float obstruction);
		float compute_velocity_linear(FusionGrid& grid, std::string layer,  
									  float maxvel, float safezone, float decay);

		float compute_angle(float x, float y);
		float compute_distance(float x, float y);
		float compute_lambda(float distance, float beta1, float beta2);
		float compute_sigma(float distance, float obstruction);

		virtual void onRunning(void);

	

	public:
		const static unsigned int ForAttractors = 1;
		const static unsigned int ForRepellors  = 2;
		const static unsigned int ForBoth       = 3;

	private:
		ros::Subscriber	rossub_attractors_;
		ros::Subscriber	rossub_repellors_;
		ros::Subscriber	rossub_odometry_;
		ros::Publisher	rospub_cmdvel_;

		FusionGrid			grid_attractors_;
		FusionGrid			grid_repellors_;
		nav_msgs::Odometry	odometry_;

		std::string 		attractors_layer_;
		float 				attractors_strength_;
		float 				attractors_decay_;
		float 				attractors_obstruction_;
		std::string 		repellors_layer_;
		float 				repellors_strength_;
		float 				repellors_decay_;
		float 				repellors_obstruction_;
		float 				velocity_;





};


	}
}

#endif
