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

	rossub_odometry_   = node->subscribe("odometry",  CNBIROS_CORE_BUFFER_MESSAGES, 
			  							 &ForceField::on_received_odometry, this);

	rospub_cmdvel_ 	  = node->advertise<geometry_msgs::Twist>("cmdvel", CNBIROS_CORE_BUFFER_MESSAGES);


	this->SetTargetLayer(CNBIROS_FORCEFIELD_TARGETLAYER, ForceField::ForBoth);
	this->SetStrength(CNBIROS_FORCEFIELD_STRENGTH, ForceField::ForBoth);
	this->SetDecay(CNBIROS_FORCEFIELD_DECAY, ForceField::ForBoth);
	this->SetObstruction(CNBIROS_FORCEFIELD_OBSTRUCTION, ForceField::ForBoth);


	this->velocity_ = CNBIROS_FORCEFIELD_VELOCITY_MAX;
}

ForceField::~ForceField(void) {}

void ForceField::SetTargetLayer(const std::string& layer, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			this->attractors_layer_ = layer;
			break;
		case ForceField::ForRepellors:
			this->repellors_layer_ = layer;
			break;
		case ForceField::ForBoth:
			this->attractors_layer_ = layer;
			this->repellors_layer_ = layer;
			break;
	}
}

void ForceField::SetStrength(float value, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			this->attractors_strength_ = value;
			break;
		case ForceField::ForRepellors:
			this->repellors_strength_ = value;
			break;
		case ForceField::ForBoth:
			this->attractors_strength_ = value;
			this->repellors_strength_ = value;
			break;
	}
}

void ForceField::SetDecay(float value, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			this->attractors_decay_ = value;
			break;
		case ForceField::ForRepellors:
			this->repellors_decay_ = value;
			break;
		case ForceField::ForBoth:
			this->attractors_decay_ = value;
			this->repellors_decay_ = value;
			break;
	}
}

void ForceField::SetObstruction(float value, unsigned int type) {
	switch(type) {
		case ForceField::ForAttractors:
			this->attractors_obstruction_ = value;
			break;
		case ForceField::ForRepellors:
			this->repellors_obstruction_ = value;
			break;
		case ForceField::ForBoth:
			this->attractors_obstruction_ = value;
			this->repellors_obstruction_ = value;
			break;
	}
}

void ForceField::on_received_attractors(const grid_map_msgs::GridMap::ConstPtr& msg) {

	FusionGrid grid;

	grid_map::GridMapRosConverter::fromMessage(*msg, grid);
	
	if(grid.Exists(this->attractors_layer_)) {
		grid[this->attractors_layer_] = this->attractors_strength_*grid[this->attractors_layer_];
		this->grid_attractors_ = grid;
	} else {
		ROS_ERROR("Target layer '%s' does not exist in incoming attractors message", 
				  this->attractors_layer_.c_str());
	}

}

void ForceField::on_received_repellors(const grid_map_msgs::GridMap::ConstPtr& msg) {

	FusionGrid grid;

	grid_map::GridMapRosConverter::fromMessage(*msg, grid);
	
	if(grid.Exists(this->repellors_layer_)) {
		grid[this->repellors_layer_] = -this->repellors_strength_*grid[this->repellors_layer_];
		this->grid_repellors_ = grid;
	} else {
		ROS_ERROR("Target layer '%s' does not exist in incoming repellors message", 
				  this->repellors_layer_.c_str());
	}
}

void ForceField::on_received_odometry(const nav_msgs::Odometry::ConstPtr& msg) {
	this->odometry_ = *msg;
}

float ForceField::compute_angle(float x, float y) {
	float angle;
	angle = -((std::atan2(y, x)) - M_PI/2.0f);

	angle = fmod(angle + M_PI, 2 * M_PI);
	return angle >=0 ? (angle-M_PI) : (angle + M_PI);
}

float ForceField::compute_distance(float x, float y) {
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

float ForceField::compute_lambda(float distance, float beta1, float beta2) {
	return beta1*exp(-(distance/beta2));
}

float ForceField::compute_sigma(float distance, float obstruction) {
	return std::atan( std::tan(M_PI/360.0f) + obstruction/(obstruction + distance));
}



float ForceField::compute_velocity_angular(FusionGrid& grid, std::string layer,  
										   float decay, float obstruction) {
	grid_map::Position 	cPosition;
	grid_map::Index 	cIndex;
	float x, y, angle, distance, value;
	float lambda, sigma;
	float fobs = 0.0f;
	
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
		value = data(cIndex(0), cIndex(1));
		
		// compute angular velocity based on attractors/repellors
		angle    = this->compute_angle(x, y);
		distance = this->compute_distance(x, y);
		lambda   = this->compute_lambda(distance, value, decay);
		sigma    = this->compute_sigma(distance, obstruction);
		sigma = 1.0f;

		//ROS_INFO("x: %f", x);
		//ROS_INFO("y: %f", y);
		//ROS_INFO("angle: %f", angle);
		//ROS_INFO("distance: %f", distance);
		//ROS_INFO("lambda: %f", lambda);
		//ROS_INFO("sigma: %f", sigma);
		fobs  += lambda*(-angle)*exp(-pow((angle),2)/(2.0f*pow(sigma, 2)));

	}

	return fobs;
}

float ForceField::compute_velocity_linear(FusionGrid& grid, std::string layer,  
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
		distance = this->compute_distance(x, y);
		angle    = this->compute_angle(x, y);

		ROS_INFO("Angle: %f", angle);
		ROS_INFO("Distance: %f", distance);

		lambda = 1.0f;
		a1 = 6.0f;
		a2 = 0.1f;
		cforce = lambda*std::exp(-(std::pow(angle, 2)/a1 + std::pow(distance, 2)/a2));

		cforce = std::max(cforce, pforce);
		pforce = cforce;
	}

	float maxinc = 0.1;
	float forceobs, forcesystem;
	forcesystem = maxinc*std::cos(this->velocity_*M_PI/2.0f);
	forceobs    = -cforce;
	velocity =  this->velocity_ + 0.01f*(forcesystem + forceobs); 
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


void ForceField::onRunning(void) {

	geometry_msgs::Twist msg;
	float force_angular = 0.0f;
	float force_linear  = 0.0f;



	force_angular += this->compute_velocity_angular(this->grid_attractors_,
													this->attractors_layer_, 
													this->attractors_decay_, 
													this->attractors_obstruction_);

	force_angular += this->compute_velocity_angular(this->grid_repellors_,
													this->repellors_layer_, 
													this->repellors_decay_, 
													this->repellors_obstruction_);
	
	force_linear  += this->compute_velocity_linear(this->grid_repellors_,
												   this->repellors_layer_, 
												   CNBIROS_FORCEFIELD_VELOCITY_MAX, 
												   CNBIROS_FORCEFIELD_VELOCITY_SAFEZONE,
												   CNBIROS_FORCEFIELD_VELOCITY_DECAY);


	//ROS_INFO("Angular velocity: %f", force_angular);
	//ROS_INFO("Linear  velocity: %f", force_linear);
	msg.linear.x = force_linear;	
	msg.linear.y = 0.0f;	
	msg.linear.z = 0.0f;	
	msg.angular.x = 0.0f;	
	msg.angular.y = 0.0f;	
	msg.angular.z = force_angular;	

	this->rospub_cmdvel_.publish(msg);
}


	}
}

#endif
