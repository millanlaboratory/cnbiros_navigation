#ifndef CNBIROS_NAVIGATION_SIMPLEFUSION_HPP
#define CNBIROS_NAVIGATION_SIMPLEFUSION_HPP

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_core/Subscribers.hpp"
#include "cnbiros_navigation/Flags.hpp"
#include "cnbiros_navigation/FusionGrid.hpp"

namespace cnbiros {
	namespace navigation {

class SimpleFusion : public cnbiros::core::NodeInterface {

	public:
		SimpleFusion(ros::NodeHandle* node, std::string name);
		virtual ~SimpleFusion(void);

		bool Add(const std::string topic, unsigned int type);
		bool Remove(const std::string topic);

		virtual void Process(void);

	private:
		void on_received_pointcloud(const sensor_msgs::PointCloud::ConstPtr& msg, std::string topic);
		void on_received_laserscan(const sensor_msgs::LaserScan::ConstPtr& msg, std::string topic);
		void on_received_point(const geometry_msgs::Point32::ConstPtr& msg, std::string topic);
		void on_received_pointcloud2(const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topic);
		
		virtual void onRunning(void);
		

	public:
		static const unsigned int AsPointCloud  = 0;
		static const unsigned int AsLaserScan   = 1;
		static const unsigned int AsPoint       = 2;
		static const unsigned int AsPointCloud2 = 3;

	private:
		cnbiros::core::Subscribers* sources_;
		ros::Publisher 				rospub_;
		FusionGrid* 				fusiongrid_;	
};



	}
}
#endif
