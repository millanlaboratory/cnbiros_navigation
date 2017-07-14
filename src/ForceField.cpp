#ifndef CNBIROS_NAVIGATION_FORCEFIELD_CPP
#define CNBIROS_NAVIGATION_FORCEFIELD_CPP

#include "cnbiros_navigation/ForceField.hpp"

namespace cnbiros {
	namespace navigation {

ForceField::ForceField(ros::NodeHandle* node, std::string name) : NodeInterface(node, name) {
	
	rossub_attractors_ = node->subscribe("attractors", CNBIROS_CORE_BUFFER_MESSAGES, 
			  							 &ForceField::on_received_attractors, this);
	
	rossub_repellors_  = node->subscribe("repellors",  CNBIROS_CORE_BUFFER_MESSAGES, 
			  							 &ForceField::on_received_repellors, this);

	rospub_cmdvel_ 	  = node->advertise<geometry_msgs::Twist>("cmd_vel", CNBIROS_CORE_BUFFER_MESSAGES);

	// Add services
	rossrv_set_strength_ = node->advertiseService(ros::this_node::getName() + "/set_strength", 
						  &ForceField::on_set_strength, this);
	rossrv_set_decay_    = node->advertiseService(ros::this_node::getName() + "/set_decay", 
						  &ForceField::on_set_decay, this);

	this->SetTargetLayer("attractors", ForceField::ForAttractors);
	this->SetTargetLayer("repellors", ForceField::ForRepellors);
	this->SetStrength(CNBIROS_FORCEFIELD_STRENGTH_ATTRACTORS, ForceField::ForAttractors);
	this->SetStrength(CNBIROS_FORCEFIELD_STRENGTH_REPELLORS, ForceField::ForRepellors);
	this->SetDecay(CNBIROS_FORCEFIELD_DECAY_ATTRACTORS, ForceField::ForAttractors);
	this->SetDecay(CNBIROS_FORCEFIELD_DECAY_REPELLORS, ForceField::ForRepellors);
	this->SetRobotSize(0.4f);

	//this->velocity_ = CNBIROS_FORCEFIELD_VELOCITY_MAX;
}

ForceField::~ForceField(void) {}

bool ForceField::on_set_strength(cnbiros_navigation::SetStrengthSrv::Request& req,
					 			 cnbiros_navigation::SetStrengthSrv::Response& res) {
	ROS_INFO("Requested to set strength");
	this->SetStrength(req.value, req.type);
	return true;
}

bool ForceField::on_set_decay(cnbiros_navigation::SetDecaySrv::Request& req,
						  	  cnbiros_navigation::SetDecaySrv::Response& res) {
	ROS_INFO("Requested to set decay");
	this->SetDecay(req.value, req.type);
	return true;
}

void ForceField::SetTargetLayer(const std::string& layer, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			ROS_INFO("Set layer for attractors to %s", layer.c_str());
			this->a_layer_ = layer;
			break;
		case ForceField::ForRepellors:
			ROS_INFO("Set layer for repellors at %s", layer.c_str());
			this->r_layer_ = layer;
			break;
		case ForceField::ForBoth:
			ROS_INFO("Set layer for attractors/repellors at %s", layer.c_str());
			this->a_layer_ = layer;
			this->r_layer_ = layer;
			break;
	}
}

void ForceField::SetStrength(float value, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			ROS_INFO("Set strength for attractors at %f", value);
			this->a_beta1_ = value;
			break;
		case ForceField::ForRepellors:
			ROS_INFO("Set strength for repellors at %f", value);
			this->r_beta1_ = value;
			break;
		case ForceField::ForBoth:
			ROS_INFO("Set strength for attractors/repellors at %f", value);
			this->a_beta1_ = value;
			this->r_beta1_ = value;
			break;
	}
}

void ForceField::SetDecay(float value, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			ROS_INFO("Set decay for attractors at %f", value);
			this->a_beta2_ = value;
			break;
		case ForceField::ForRepellors:
			ROS_INFO("Set decay for repellors at %f", value);
			this->r_beta2_ = value;
			break;
		case ForceField::ForBoth:
			ROS_INFO("Set decay for attractors/repellors at %f", value);
			this->a_beta2_ = value;
			this->r_beta2_ = value;
			break;
	}
}

void ForceField::SetRobotSize(float size) {
	this->robot_size_ = size;
}

void ForceField::on_received_attractors(const grid_map_msgs::GridMap::ConstPtr& msg) {
	
	float beta1, beta2;
	std::string layer;

	layer = this->a_layer_;
	beta1 = this->a_beta1_;
	beta2 = this->a_beta2_;
	grid_map::GridMapRosConverter::fromMessage(*msg, this->a_grid_);

}

void ForceField::on_received_repellors(const grid_map_msgs::GridMap::ConstPtr& msg) {
	float beta1, beta2;
	std::string layer;

	layer = this->r_layer_;
	beta1 = this->r_beta1_;
	beta2 = this->r_beta2_;
	grid_map::GridMapRosConverter::fromMessage(*msg, this->r_grid_);

}

float ForceField::compute_angular_velocity(grid_map::GridMap& grid, std::string layer,  
										   float beta1, float beta2) {
	grid_map::Position 	cPosition;
	grid_map::Index 	cIndex;
	float posx, posy, posv;
	float theta, dist;
	float lambda, sigma;
	float robotsize;
	float fobs;

	fobs = 0.0f;
	robotsize = this->robot_size_;

	if(grid.exists(layer) == false) {
		ROS_WARN_ONCE("Target layer '%s' does not exist, cannot compute angular velocity", layer.c_str()); 
		return 0.0f;
	}
	
	grid_map::Matrix& data = grid[layer];	

	for(grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {

		cIndex = grid_map::Index(*it);

		if(data(cIndex(0), cIndex(1)) == 0)
			continue;

		// get current position
		grid.getPosition(cIndex, cPosition);

		// exclude the positions on the back
		if(cPosition.x() < -0.1f)
			continue;

		// get x and y cohordinates (reverse for standard usage)
		posx = -cPosition.y();
		posy =  cPosition.x();
		posv = data(cIndex(0), cIndex(1));
		
		// compute angular velocity based on attractors/repellors
		theta    = TrigTools::Angle(posx, posy);
		dist     = TrigTools::Radius(posx, posy);
		lambda   = beta1*exp(-(dist/beta2));
		sigma    = TrigTools::AngleNorm(std::atan(std::tan(M_PI/360.0f)+robotsize/(robotsize + dist)));
		
		fobs += lambda*(M_PI/2.0f-theta)*exp(-pow(M_PI/2.0f-theta,2)/(2.0f*pow(sigma, 2)));
	}

	//printf("fobs: %f\n", fobs);

	return fobs;
}

/*
float ForceField::compute_velocity_linear(fusion::FusionGrid& grid, std::string layer,  
										   float maxvel, float safezone, float decay) {
	grid_map::Position 	cPosition;
	grid_map::Index 	cIndex;
	float x, y, angle;
	float fobs = 0.0f;
	float velocity;
	float distance;
	float pforce = 0.0f;
	float cforce = 0.0f;
	float a1, a2;
	float lambda;

	if(grid.Exists(layer) == false) {
		ROS_ERROR("Target layer '%s' does not exist, cannot compute angular velocity", layer.c_str()); 
		return 0.0f;
	}
	
	grid_map::Matrix& data = grid[layer];	


	for(grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {

		cIndex = grid_map::Index(*it);

		if(data(cIndex(0), cIndex(1)) == 0)
			continue;

		// get current position
		grid.getPosition(cIndex, cPosition);

		// exclude the positions on the back
		if(cPosition.x() < -0.1f)
			continue;

		// get x and y cohordinates (reverse for standard usage)
		x = -cPosition.y();
		y =  cPosition.x();
		angle    = TrigTools::Angle(x, y);
		distance = TrigTools::Radius(x, y);

		ROS_INFO("Angle: %f", angle);
		ROS_INFO("Distance: %f", distance);

		lambda = 1.0f;
		a1 = 6.0f;
		a2 = 0.04f;
		cforce = lambda*std::exp(-(std::pow(angle, 2)/a1 + std::pow(distance, 2)/a2));

		cforce = std::max(cforce, pforce);
		pforce = cforce;
	}

	float maxinc = 0.1;
	float forceobs, forcesystem;
	forcesystem = maxinc*std::cos(this->velocity_/maxvel*M_PI/2.0f);
	forceobs    = -cforce;
	velocity =  this->velocity_ + 0.01f*(forcesystem + 1.1f*forceobs); 
	if(velocity >= CNBIROS_FORCEFIELD_VELOCITY_MAX)
		velocity = CNBIROS_FORCEFIELD_VELOCITY_MAX;
	if(velocity <= 0.0f)
		velocity = 0.01f;
	
	ROS_INFO("forceobs: %f", forceobs);
	ROS_INFO("forcesystem: %f", forcesystem);
	ROS_INFO("velocity: %f", velocity);
	this->velocity_ = velocity;

	return velocity;
}

*/
void ForceField::onRunning(void) {

	geometry_msgs::Twist msg;
	float force_angular = 0.0f;
	//float force_linear  = 0.0f;

	force_angular -= this->compute_angular_velocity(this->a_grid_, 
													this->a_layer_, this->a_beta1_, this->a_beta2_);
	force_angular += this->compute_angular_velocity(this->r_grid_, 
													this->r_layer_, this->r_beta1_, this->r_beta2_);

	msg.linear.x = 0.0f;	
	msg.linear.y = 0.0f;	
	msg.linear.z = 0.0f;	
	msg.angular.x = 0.0f;	
	msg.angular.y = 0.0f;	
	msg.angular.z = force_angular;	
	this->rospub_cmdvel_.publish(msg);

	//force_angular += this->compute_velocity_angular(*(this->agrid_),
	//												this->alayer_, 
	//												this->adecay_, 
	//												1.0f);

	//force_angular += this->compute_velocity_angular(*(this->rgrid_),
	//												this->rlayer_, 
	//												this->rdecay_, 
	//												this->robstruction_);
	//
	//force_linear  += this->compute_velocity_linear(*(this->rgrid_),
	//											   this->rlayer_, 
	//											   CNBIROS_FORCEFIELD_VELOCITY_MAX, 
	//											   CNBIROS_FORCEFIELD_VELOCITY_SAFEZONE,
	//											   CNBIROS_FORCEFIELD_VELOCITY_DECAY);


	////ROS_INFO("Angular velocity: %f", force_angular);
	////ROS_INFO("Linear  velocity: %f", force_linear);
	//msg.linear.x = force_linear;	
	//msg.linear.y = 0.0f;	
	//msg.linear.z = 0.0f;	
	//msg.angular.x = 0.0f;	
	//msg.angular.y = 0.0f;	
	//msg.angular.z = force_angular;	

	//this->rospub_cmdvel_.publish(msg);
}


//float ForceField::compute_angle(float x, float y) {
//	return AngleTools::Norm(((std::atan2(y, x)) - M_PI/2.0f));
//}

//float ForceField::compute_distance(float x, float y) {
//	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
//}
	}
}

#endif
