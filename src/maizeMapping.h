/*
 * maizeMapping.h
 *
 *  	Created on: Jun 10, 2015
 *      Author: Marat "Peter" Purnyn
 *	
 *	Created for use on Cornholio for the Field Robotics Event 2015
 *	This program uses odometry data (/odom), laserscan data (/scan), & Fx8 Infrared and Distance  
 *	images
 */

#ifndef LS2IMAGE_H_
#define LS2IMAGE_H_

// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

// Marker
#include <visualization_msgs/Marker.h>

// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// Standard Libraries
#include <sys/timeb.h>
#include <cmath>

//Transform
#include <tf/transform_broadcaster.h>

class MaizeMapping {
public:
		MaizeMapping();
		virtual ~MaizeMapping();

		void init();
		void loop();
private:
		

		//ROS
		ros::NodeHandle n;

		//Messages
		sensor_msgs::LaserScan laserscan;
		std_msgs::Header header;
		std::string encoding;
		std::string frame_id_marker;
		geometry_msgs::PoseWithCovarianceStamped odom;
		int lPlantData;
		int rPlantData;
		int navStateData;

		// To publish
		
		// Timer
		ros::Time laserscanner_time, laserscanner_time_old;
		timeb TimePublishRoiImageStart, TimePublishRoiImageEnd;
		int TimePublishRoiImagems;

		// To subscribe
		ros::Subscriber laserscan_sub;
		ros::Subscriber odom_sub;
		ros::Subscriber lPlant_sub;
		ros::Subscriber rPlant_sub;
		ros::Subscriber nav_state_sub;

		// Map
		int sequence;
		double rowCount;
		double rowLength;
		double mapResolution;
		double mapPadding;
		double plantSpacing;
		double rowSpacing;
		bool pathL2R;
		image_transport::ImageTransport it_;
		image_transport::Publisher maize_map_pub_;

		// Laserscanner
		double scanner_minimal_distance;
		double scan_angle_min;
		double scan_angle_max;
		double scan_angle_sim_min;
		double scan_angle_sim_max;

		//Functions
		void generateMap();
		int detectLeftPlant();
		int detectRightPlant();
		int getNavState();
		
		//Callbacks
		void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg);
		void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg);
		void lPlantCallback(const std_msgs::Int8::ConstPtr& l_plant_msg);
		void rPlantCallback(const std_msgs::Int8::ConstPtr& r_plant_msg);
		void navStateCallback(const std_msgs::UInt8::ConstPtr& r_plant_msg);
};

#endif /* LS2IMAGE_H_ */
