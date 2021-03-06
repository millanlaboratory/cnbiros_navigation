#ifndef CNBIROS_NAVIGATION_FORCEFIELD_CPP
#define CNBIROS_NAVIGATION_FORCEFIELD_CPP

#include "cnbiros_navigation/ForceField.hpp"

namespace cnbiros {
	namespace navigation {

ForceField::ForceField(ros::NodeHandle* node, std::string name) : NodeInterface(node, name) {
	
	rossub_attractors_ = node->subscribe("attractors", CNBIROS_CORE_BUFFER_MESSAGES, 
			  							 &ForceField::on_received_attractors, this);
	
	rossub_repellors_  = node->subscribe("repellors", CNBIROS_CORE_BUFFER_MESSAGES, 
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
	this->SetRobotSize(CNBIROS_FORCEFIELD_ROBOT_SIZE, CNBIROS_FORCEFIELD_ROBOT_SECTOR);

	unsigned int n_sectors = CNBIROS_FORCEFIELD_NUMBER_SECTORS;
	this->r_sectors_ = std::vector<float> (n_sectors,std::numeric_limits<float>::infinity());
	this->a_sectors_ = std::vector<float> (n_sectors,std::numeric_limits<float>::infinity());

	this->Stop();
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

void ForceField::SetRobotSize(float size, float sector) {
	this->robot_size_ = size;
	this->robot_sector_ = sector;
	ROS_INFO("Set robot size to %f [m]", size);
	ROS_INFO("Set robot sensor sector to %f [rad]", sector);
}

void ForceField::on_received_attractors(const grid_map_msgs::GridMap::ConstPtr& msg) {
	//printf("received attractors");
	
	float beta1, beta2;
	std::string layer;

	layer = this->a_layer_;
	beta1 = this->a_beta1_;
	beta2 = this->a_beta2_;
	grid_map::GridMapRosConverter::fromMessage(*msg, this->a_grid_);
	this->convert_grid_to_sector(this->a_grid_, this->a_layer_, this->a_sectors_);

}

void ForceField::on_received_repellors(const grid_map_msgs::GridMap::ConstPtr& msg) {
	//printf("received repellors");
	
	float beta1, beta2;
	std::string layer;

	layer = this->r_layer_;
	beta1 = this->r_beta1_;
	beta2 = this->r_beta2_;
	grid_map::GridMapRosConverter::fromMessage(*msg, this->r_grid_);
	this->convert_grid_to_sector(this->r_grid_, this->r_layer_, this->r_sectors_);

}



void ForceField::convert_grid_to_sector(grid_map::GridMap& grid, std::string layer,  
										   std::vector<float>& sectors) {
	grid_map::Position 	cPosition;
	grid_map::Index 	cIndex;
	float posx, posy, posv;
	float theta, dist;
	unsigned int ind;
	unsigned int n_sectors;

	n_sectors = sectors.size();

	if(grid.exists(layer) == false) {
		ROS_WARN_ONCE("Target layer '%s' does not exist, cannot compute angular velocity", layer.c_str()); 
		return;
	}
	
	grid_map::Matrix& data = grid[layer];	

	sectors = std::vector<float> (n_sectors,std::numeric_limits<float>::infinity()); 

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
		// min ensures that ind < n_sectors (index would be out of range for theta==M_PI)
		ind   	 = std::min((unsigned int)(std::floor(n_sectors*(theta/M_PI))), n_sectors-1);
		sectors[ind] = std::min(sectors[ind],dist);
	}

}

float ForceField::compute_angular_velocity_repellors(std::vector<float>& sectors, float beta1, float beta2)	 {
	float fobs;
	float distance;
	float lambda;
	float sigma;
	float robotsize, robotsector;
	float theta;
	unsigned int index;
	
	fobs = 0.0f;
	robotsize   = this->robot_size_;
	robotsector = this->robot_sector_;

	for(std::vector<float>::iterator it = sectors.begin(); it != sectors.end(); ++it){		

		if (!(std::isfinite(*it)))
			continue;
		
		distance = *it;
		index 	 = it-sectors.begin();
		theta    = M_PI/sectors.size()*(index+0.5f);
		lambda   = beta1*exp(-(distance/beta2));
		sigma    = TrigTools::AngleNorm(std::atan(std::tan(robotsector/2.0f)+robotsize/(robotsize + distance)));
		fobs += lambda*(M_PI/2.0f-theta)*exp(-pow(M_PI/2.0f-theta,2)/(2.0f*pow(sigma, 2)));
	}

	return fobs;
}

float ForceField::compute_angular_velocity_attractors(std::vector<float>& sectors, float beta1, float beta2)	 {
	float fobs;
	float distance;
	float lambda;
	float sigma;
	float robotsize, robotsector;
	float theta;
	unsigned int index;
	
	fobs = 0.0f;
	robotsize   = this->robot_size_;
	robotsector = this->robot_sector_;

	for(std::vector<float>::iterator it = sectors.begin(); it != sectors.end(); ++it){		

		if (!(std::isfinite(*it)))
			continue;
		
		distance = *it;
		index 	 = it-sectors.begin();
		theta    = M_PI/sectors.size()*(index+0.5f);
		lambda   = beta1*exp(-(distance/beta2));
		sigma    = TrigTools::AngleNorm(std::atan(std::tan(robotsector/2.0f)+robotsize/(robotsize + distance)));
		fobs += lambda*(M_PI/2.0f-theta);
	}

	return fobs;
}



float ForceField::compute_linear_velocity(std::vector<float>& sectors, float maxvel, 
								float safezone, float decay, float audacity) {
	//do not necessarily need safezone - not used at the moment
	float velocity;
	float distance;
	float lambda;
	float x_distance_center;
	float y_distance_front;
	float robotradius;
	float theta;
	unsigned int index;

	robotradius = 0.3f*this->robot_size_+safezone;
	
	velocity  = maxvel;
	
	for(std::vector<float>::iterator it = sectors.begin(); it != sectors.end(); ++it){
		distance = *it;
		
		//infinite distance -> no detected obstacle in sector, don't reduce velocity
		if(!(std::isfinite(distance)))
			continue;
		
		index = it-sectors.begin();
		theta = M_PI/sectors.size()*(index+0.5f);
		
		//x-projection of distance to center of robot
		x_distance_center = std::abs(std::cos(theta)*(distance));
		
		//y-projection of distance to front (+safezone) of robot
		y_distance_front = std::sin(theta)*(distance);
		
		if(x_distance_center <= robotradius){
			y_distance_front = std::max(
							y_distance_front - std::sqrt(std::pow(robotradius,2.0f)-std::pow(x_distance_center,2.0f)), 0.01f);
		}
		else {
			//y_distance_front = std::max(
			//				y_distance_front*std::exp(audacity/robotradius*std::pow((x_distance_center - robotradius),2.0f)),
			//				y_distance_front+std::sqrt(audacity)*(x_distance_center-robotradius));
			y_distance_front = y_distance_front+std::exp(audacity/robotradius*std::pow((x_distance_center - robotradius),2.0f))-1.0f;
		}
		velocity = std::min(velocity, maxvel*std::exp(-decay/y_distance_front));
	}

	//velocity = std::max(0.01f, std::min(maxvel, velocity));


	return velocity;

}
/*
float ForceField::compute_linear_velocity(fusion::FusionGrid& grid, std::string layer,  
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

void ForceField::onStraight(void) {

	geometry_msgs::Twist msg;
	
	msg.linear.x = CNBIROS_FORCEFIELD_VELOCITY_MAX;	
	msg.linear.y = 0.0f;	
	msg.linear.z = 0.0f;	
	msg.angular.x = 0.0f;	
	msg.angular.y = 0.0f;	
	msg.angular.z = 0.0f;	
	
	this->rospub_cmdvel_.publish(msg);
}

void ForceField::onLeft(void) {

	geometry_msgs::Twist msg;
	
	msg.linear.x = 0.0f;	
	msg.linear.y = 0.0f;	
	msg.linear.z = 0.0f;	
	msg.angular.x = 0.0f;	
	msg.angular.y = 0.0f;	
	msg.angular.z = 1.0f;	
	
	this->rospub_cmdvel_.publish(msg);
}

void ForceField::onRight(void) {

	geometry_msgs::Twist msg;

	msg.linear.x = 0.0f;	
	msg.linear.y = 0.0f;	
	msg.linear.z = 0.0f;	
	msg.angular.x = 0.0f;	
	msg.angular.y = 0.0f;	
	msg.angular.z = -1.0f;	
	
	this->rospub_cmdvel_.publish(msg);
}


void ForceField::onRunning(void) {

	geometry_msgs::Twist msg;
	float force_angular = 0.0f;
	float velocity_linear = 0.0f;
	

	this->convert_grid_to_sector(this->a_grid_, this->a_layer_, this->a_sectors_);
	this->convert_grid_to_sector(this->r_grid_, this->r_layer_, this->r_sectors_);
		
	force_angular -= this->compute_angular_velocity_attractors(this->a_sectors_, this->a_beta1_, this->a_beta2_);
	force_angular += this->compute_angular_velocity_attractors(this->r_sectors_, this->r_beta1_, this->r_beta2_);
	velocity_linear = this->compute_linear_velocity(this->r_sectors_, CNBIROS_FORCEFIELD_VELOCITY_MAX, 
													   CNBIROS_FORCEFIELD_VELOCITY_SAFEZONE, CNBIROS_FORCEFIELD_VELOCITY_DECAY,
													   CNBIROS_FORCEFIELD_VELOCITY_AUDACITY);


	msg.linear.x = velocity_linear;	
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
	//force_linear  += this->compute_linear_velocity(*(this->rgrid_),
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
