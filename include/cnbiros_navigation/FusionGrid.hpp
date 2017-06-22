#ifndef CNBIROS_NAVIGATION_FUSIONGRID_HPP
#define CNBIROS_NAVIGATION_FUSIONGRID_HPP

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

namespace cnbiros {
	namespace navigation {

class FusionGrid : public grid_map::GridMap {

	public:
		FusionGrid(void);
		FusionGrid(const float x, const float y, const float r);
		~FusionGrid(void);

		void AddLayer(const std::string& layer);
		bool RemoveLayer(const std::string& layer);
		bool Exists(const std::string& layer);
		void SetGeometry(const float x, const float y, const float r);
		grid_map_msgs::GridMap ToMessage(void);
		
		void SetFrame(const std::string& frame);
		std::string GetFrame(void);
		void Reset(const std::string& layer, float value = 0.0f);
		void Reset(float value = 0.0f);
		
		bool Sum(const std::string& target);
		bool Sum(const std::string& target, const std::vector<std::string>& layers);


		bool ReplaceNaN(const std::string& layer, float value = 0.0f);
		bool ReplaceNaN(float value = 0.0f);

		bool SetMin(const std::string& layer, float minimum);
		bool SetMax(const std::string& layer, float maximum);
		bool SetMinMax(const std::string& layer, float minimum, float maximum);


		void Update(const std::string& layer, sensor_msgs::LaserScan& msg, float radius = 0.0f);
		void Update(const std::string& layer, sensor_msgs::PointCloud& msg);
		void Update(const std::string& layer, geometry_msgs::Point32& msg);
		void Update(const std::string& layer, grid_map::Matrix& data);


};

	}
}


#endif
