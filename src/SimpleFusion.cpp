#ifndef CNBIROS_NAVIGATION_SIMPLEFUSION_CPP
#define CNBIROS_NAVIGATION_SIMPLEFUSION_CPP

#include "cnbiros_navigation/SimpleFusion.hpp"

namespace cnbiros {
	namespace navigation {

SimpleFusion::SimpleFusion(ros::NodeHandle* node, std::string name) : cnbiros::core::NodeInterface(node, name) {
	this->sources_ = new cnbiros::core::Subscribers(node);
	
	this->rospub_ = node->advertise<grid_map_msgs::GridMap>
						(node->getNamespace()+"/"+name, CNBIROS_CORE_BUFFER_MESSAGES);
	
	this->rossrv_reset_ = node->advertiseService(
								  node->getNamespace()+"/"+name+"/reset", 
								  &SimpleFusion::on_reset_service, this);
	
	this->fusiongrid_ = new FusionGrid(CNBIROS_NAVIGATION_FUSIONGRID_X,
									   CNBIROS_NAVIGATION_FUSIONGRID_Y,
									   CNBIROS_NAVIGATION_FUSIONGRID_R);
	this->fusiongrid_->AddLayer("fusion");
	this->fusiongrid_->SetFrame("base_link");
	this->fusiongrid_->Reset();
}

SimpleFusion::~SimpleFusion(void) {
	delete this->sources_;
}

bool SimpleFusion::on_reset_service(cnbiros_navigation::ResetGridService::Request& req,
									cnbiros_navigation::ResetGridService::Response& res) {

	ROS_INFO("Requested to reset");
	res.result = true;
	if(this->fusiongrid_->Exists(req.layer)) {
		this->fusiongrid_->Reset(req.layer);
	} else {
		res.result = false;
	}

	return res.result;

}

bool SimpleFusion::Remove(const std::string topic) {
	return this->sources_->Remove(topic);
}

bool SimpleFusion::Add(const std::string topic, unsigned int type) {

	bool retcode = false;
	switch(type) {
		case SimpleFusion::AsPointCloud:
			ROS_INFO("Topic %s added as source (PointCloud)", topic.c_str());
			retcode = this->sources_->Add<sensor_msgs::PointCloud>(topic, 
					  boost::bind(&SimpleFusion::on_received_pointcloud, this, _1, topic));
			break;
		case SimpleFusion::AsLaserScan:
			ROS_INFO("Topic %s added as source (LaserScan)", topic.c_str());
			retcode = this->sources_->Add<sensor_msgs::LaserScan>(topic, 
					  boost::bind(&SimpleFusion::on_received_laserscan, this, _1, topic));
			break;
		case SimpleFusion::AsPoint:
			ROS_INFO("Topic %s added as source (Point)", topic.c_str());
			retcode = this->sources_->Add<geometry_msgs::Point32>(topic, 
					  boost::bind(&SimpleFusion::on_received_point, this, _1, topic));
			break;
		case SimpleFusion::AsPointCloud2:
			ROS_INFO("Topic %s added as source (PointCloud2)", topic.c_str());
			retcode = this->sources_->Add<sensor_msgs::PointCloud2>(topic, 
					  boost::bind(&SimpleFusion::on_received_pointcloud2, this, _1, topic));
			break;
		default:
			break;
	}

	this->fusiongrid_->AddLayer(topic);
	this->fusiongrid_->Reset(topic);

	return retcode;
}

void SimpleFusion::on_received_pointcloud(const sensor_msgs::PointCloud::ConstPtr& msg, std::string topic) {
	sensor_msgs::PointCloud data = *msg;
	this->fusiongrid_->Reset(topic);
	this->fusiongrid_->Update(topic, data);
}

void SimpleFusion::on_received_laserscan(const sensor_msgs::LaserScan::ConstPtr& msg, std::string topic) {
	sensor_msgs::LaserScan data = *msg;
	this->fusiongrid_->Reset(topic);
	this->fusiongrid_->Update(topic, data);
}

void SimpleFusion::on_received_point(const geometry_msgs::Point32::ConstPtr& msg, std::string topic) {
	geometry_msgs::Point32 data = *msg;
	this->fusiongrid_->Reset(topic);
	this->fusiongrid_->Update(topic, data);
}

void SimpleFusion::on_received_pointcloud2(const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topic) {
	ROS_WARN("Not implemented");
}


void SimpleFusion::Process(void) {

	grid_map_msgs::GridMap msg;

	
	this->fusiongrid_->Reset("fusion");
	this->fusiongrid_->Sum("fusion");
	this->fusiongrid_->SetMax("fusion", 1.0f);
	this->rospub_.publish(this->fusiongrid_->ToMessage());
}

void SimpleFusion::onRunning(void) {
	this->Process();
}

	}
}


#endif
