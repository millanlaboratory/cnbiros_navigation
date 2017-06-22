#ifndef CNBIROS_NAVIGATION_FUSIONGRID_CPP
#define CNBIROS_NAVIGATION_FUSIONGRID_CPP

#include "cnbiros_navigation/FusionGrid.hpp"

namespace cnbiros {
	namespace navigation {

FusionGrid::FusionGrid(void) {}

FusionGrid::FusionGrid(const float x, const float y, const float r) {
	this->SetGeometry(x, y, r);
}

FusionGrid::~FusionGrid(void) {}

void FusionGrid::AddLayer(const std::string& layer) {
	this->add(layer);
}

bool FusionGrid::RemoveLayer(const std::string& layer) {
	return this->erase(layer);
}

bool FusionGrid::Exists(const std::string& layer) {
	return this->exists(layer);
}

void FusionGrid::SetGeometry(const float x, const float y, const float r) {
	this->setGeometry(grid_map::Length(x, y), r);
}

grid_map_msgs::GridMap FusionGrid::ToMessage(void) {
	grid_map_msgs::GridMap msg;
	grid_map::GridMapRosConverter::toMessage(*this, msg);
	return msg;
}

void FusionGrid::SetFrame(const std::string& frame) {
	this->setFrameId(frame);
}

std::string FusionGrid::GetFrame(void) {
	return this->getFrameId();
}

void FusionGrid::Reset(const std::string& layer, float value) {

	if(this->Exists(layer)) {
		this->get(layer).setConstant(value);
	}
}

void FusionGrid::Reset(float value) {

	std::vector<std::string> layers = this->getLayers();
	
	for(auto it = layers.begin(); it != layers.end(); ++it) {
		this->Reset(*it, value);
	}
}

bool FusionGrid::Sum(const std::string& target) {
	bool result;
	std::vector<std::string> layers;
	layers = this->getLayers();

	result = this->Sum(target, layers);
	
	return result;
}

bool FusionGrid::Sum(const std::string& target, const std::vector<std::string>& layers) {
	bool result = true;
	std::vector<std::string>::const_iterator it;

	if(this->Exists(target) == false) {
		ROS_WARN("Target layer (%s) does not exist. Cannot sum required layers", target.c_str());
		return false;
	}

	for(it = layers.begin(); it != layers.end(); ++it) {
		if(this->Exists(*it) == true) {
			if((*it).compare(target) != 0) {
				this->get(target) += this->get(*it);
			}
		}
	}

	return result;
}

bool FusionGrid::ReplaceNaN(const std::string& layer, float value) {

	bool result = false;
	if(this->Exists(layer)) {
		this->get(layer) = (this->get(layer).array().isNaN()).select(0.0f, this->get(layer)); 	
		result = true;
	}

	return result;
}

bool FusionGrid::ReplaceNaN(float value) {

	bool result = true;
	std::vector<std::string> layers;
	std::vector<std::string>::iterator it;
	
	layers = this->getLayers();

	for (it = layers.begin(); it != layers.end(); ++it) {
		result = result & this->ReplaceNaN(*it, value);
	}
	
	return result;
}

bool FusionGrid::SetMin(const std::string& layer, float minimum) {
	bool result = false;

	if(this->Exists(layer) == true) {
		this->get(layer) = (this->get(layer).array() < minimum).select(minimum, this->get(layer));
		result = true;
	}

	return result;
}

bool FusionGrid::SetMax(const std::string& layer, float maximum) {
	bool result = false;

	if(this->Exists(layer) == true) {
		this->get(layer) = (this->get(layer).array() > maximum).select(maximum, this->get(layer));
		result = true;
	}

	return result;
}

bool FusionGrid::SetMinMax(const std::string& layer, float minimum, float maximum) {
	return this->SetMin(layer, minimum) & this->SetMax(layer, maximum);
}

void FusionGrid::Update(const std::string& layer, sensor_msgs::LaserScan& msg, float radius) {

	float angle, x, y;
	std::vector<float>::iterator itr;
	std::vector<float>::iterator iti;

	angle = msg.angle_min;

	if(msg.intensities.empty()) {
		msg.intensities = std::vector<float>(msg.ranges.size(), 1.0f);
	}

	iti = msg.intensities.begin();
	
	for (itr = msg.ranges.begin(); itr != msg.ranges.end(); ++itr) {
					
		// Skip ranges with inf value
		if(std::isinf(*itr) == false) {
			
			// Get cartesian cohordinates -> To be added: position of the kinect
			x = (*itr + radius)*cos(angle);
			y = (*itr + radius)*sin(angle);

			grid_map::Position position(x, y);

			// Skip positions outside the grid range
			if(this->isInside(position)) {
					
				// Fill the grid cell if ranges are between the min/max limits
				if ( ((*itr) > msg.range_min) & ((*itr) < msg.range_max)) {
					this->atPosition(layer, position) = *iti;
				} else {
					this->atPosition(layer, position) = 0.0f;
				}

			}
		}
		
		angle += msg.angle_increment;
		iti++;
	}
}

void FusionGrid::Update(const std::string& layer, sensor_msgs::PointCloud& msg) {

	grid_map::Position position;
	grid_map::Index    index;

	for(auto it = msg.points.begin(); it != msg.points.end(); ++it) {
	
		position = grid_map::Position((*it).x, (*it).y);

		if(this->isInside(position)) {
			this->atPosition(layer, position) = 1.0f;
		}
	}
}

void FusionGrid::Update(const std::string& layer, geometry_msgs::Point32& msg) {

	grid_map::Position position;
	position = grid_map::Position(msg.x, msg.y);
	
	if(this->isInside(position)) {
		this->atPosition(layer, position) = 1.0f;
	}
}

void FusionGrid::Update(const std::string& layer, grid_map::Matrix& data) {
}

	}
}



#endif
