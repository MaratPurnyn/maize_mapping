/*
 * maizeMapping.cpp
 *
 *  	Created on: Jun 10, 2015
 *      Author: Marat "Peter" Purnyn
 *	
 *	Created for use on Cornholio for the Field Robotics Event 2015
 *	This program uses odometry data (/odom), laserscan data (/scan), & Fx8 Infrared and Distance  
 *	images
 */

#include "maizeMapping.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <string>

using namespace cv;

MaizeMapping::MaizeMapping()
: it_(n)
{
	// TODO Auto-generated constructor stub
	sequence = 0;
}

MaizeMapping::~MaizeMapping() {
	// TODO Auto-generated destructor stub
}

/*********************************** Callbacks ******************************************************************/
/****************************************************************************************************************/

void MaizeMapping::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg){
	laserscan = *laserscan_msg;
}

void MaizeMapping::odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg){
  	odom = *odom_msg;
}

void MaizeMapping::lPlantCallback(const std_msgs::Int8::ConstPtr& l_plant_msg){
	if(l_plant_msg->data == 1){
		lPlantData = 1;	//green plant
	}
	else if(l_plant_msg->data == 3){
		lPlantData = 3; //brown plant
	}
}

void MaizeMapping::rPlantCallback(const std_msgs::Int8::ConstPtr& r_plant_msg){
	if(r_plant_msg->data == 1){
		rPlantData = 1;	//green plant
	}
	else if(r_plant_msg->data == 3){
		rPlantData = 3; //brown plant
	}
}

void MaizeMapping::navStateCallback(const std_msgs::UInt8::ConstPtr& nav_state_msg){
	if(nav_state_msg->data == 1){ 
		navStateData = 1; //end of row
	}
	else if(nav_state_msg->data == 9){ 
		navStateData = 9; //start of row
	}
	else if(nav_state_msg->data == 3 || nav_state_msg->data == 4){ 
		navStateData = 3; //turn
	}

}

/*********************************** /Callbacks ******************************************************************/
/****************************************************************************************************************/

void MaizeMapping::init()
{
	// Parameters can be optionally defined in the launch file to override what's hard-coded here.

	// frame_id_marker
	frame_id_marker = "/laserscan_link";
	if(ros::param::get("~frame_id_marker", frame_id_marker)){}
	else ros::param::set("~frame_id_marker", frame_id_marker);

	// rowCount - # of Rows
	rowCount = 6;
	if(ros::param::get("~rowCount", rowCount)){}
	else ros::param::set("~rowCount", rowCount);

	// rowLength (meters)
	rowLength = 10;
	if(ros::param::get("~rowLength", rowLength)){}
	else ros::param::set("~rowLength", rowLength);

	// mapResolution (pixels per meter)
	mapResolution = 200;
	if(ros::param::get("~mapResolution", mapResolution)){}
	else ros::param::set("~mapResolution", mapResolution);

	// mapPadding (pixels)
	mapPadding = 100;
	if(ros::param::get("~mapPadding", mapResolution)){}
	else ros::param::set("~mapPadding", mapResolution);

	// plantSpacing (meters)
	plantSpacing = 0.01;
	if(ros::param::get("~plantSpacing", plantSpacing)){}
	else ros::param::set("~plantSpacing", plantSpacing);

	// rowSpacing (meters)
	rowSpacing = 3;
	if(ros::param::get("~rowSpacing", rowSpacing)){}
	else ros::param::set("~rowSpacing", rowSpacing);

	// pathL2R - boolean that indicates if the robot navigates left to right or the reverse
	pathL2R = true;
	if(ros::param::get("~pathL2R", pathL2R)){}
	else ros::param::set("~pathL2R", pathL2R);

	// scan_angle_min
	scan_angle_min = -85;
	if(ros::param::get("~scan_angle_min", scan_angle_min)){}
	else ros::param::set("~scan_angle_min", scan_angle_min);

	// scan_angle_max
	scan_angle_max = 85;
	if(ros::param::get("~scan_angle_max", scan_angle_max)){}
	else ros::param::set("~scan_angle_max", scan_angle_max);

	// scan_angle_sim_min
	scan_angle_sim_min = -85;
	if(ros::param::get("~scan_angle_sim_min", scan_angle_sim_min)){}
	else ros::param::set("~scan_angle_sim_min", scan_angle_sim_min);

	// angle_sim_max
	scan_angle_sim_max = 85;
	if(ros::param::get("~scan_angle_sim_max", scan_angle_sim_max)){}
	else ros::param::set("~scan_angle_sim_max", scan_angle_sim_max);
	if(ros::param::has("/misc_simulate"))
	{
		scan_angle_max = scan_angle_sim_max * (M_PI/180.0);
		scan_angle_min = scan_angle_sim_min * (M_PI/180.0);
	}
	else
	{
		scan_angle_max = scan_angle_max * (M_PI/180.0);
		scan_angle_min = scan_angle_min * (M_PI/180.0);
	}

	// To subscribe
	laserscan_sub = n.subscribe("scan", 1, &MaizeMapping::laserscanCallback, this);
	odom_sub = n.subscribe("/odom_combined", 1, &MaizeMapping::odomCallback, this);
	lPlant_sub = n.subscribe("/fx8Left/plantDetection/plantStatusDetect", 1, &MaizeMapping::lPlantCallback, this);
	rPlant_sub = n.subscribe("/fx8Right/plantDetection/plantStatusDetect", 1, &MaizeMapping::rPlantCallback, this);
	nav_state_sub = n.subscribe("/nav_state", 10, &MaizeMapping::navStateCallback, this);

	// To publish
	maize_map_pub_ = it_.advertise("maizeMap", 1);

	//print the parameters
	printf(
		"rowCount = %d  rowLength = %.2fm  rowSpacing = %.2fm \n",
		(int)(rowCount), (double)(rowLength), (double)(rowSpacing)
	);
	cv::startWindowThread();
}

int MaizeMapping::detectLeftPlant(){
	ros::spinOnce();
	if(!laserscan.header.stamp.is_zero())
	{
		int i = fabs((laserscan.angle_min - scan_angle_min) / laserscan.angle_increment);
		for(float d = scan_angle_min; d < scan_angle_max - laserscan.angle_increment; d+=laserscan.angle_increment, i++)
		{
			if(laserscan.ranges[i] > 0.1 && laserscan.ranges[i] < 1)
			{
				//printf("allleft: ranges[i] = %f\n",laserscan.ranges[i]);
				if(d > -1.5 && d < -1.4){
					if(lPlantData == 3){
						return 3;
						printf("left brown");
					}
					return 1;
					//printf("left: ranges[i] = %f\n",laserscan.ranges[i]);
				}
			}
		}
	}
	return 0;
}

int MaizeMapping::detectRightPlant(){
	ros::spinOnce();

	if(!laserscan.header.stamp.is_zero())
	{
		int i = fabs((laserscan.angle_min - scan_angle_min) / laserscan.angle_increment);
		for(float d = scan_angle_min; d < scan_angle_max - laserscan.angle_increment; d+=laserscan.angle_increment, i++)
		{
			if(laserscan.ranges[i] > 0.1 && laserscan.ranges[i] < 1)
			{
				if(d < 1.5 && d > 1.4){
					if(rPlantData == 3){
						return 3;
						printf("right brown");
					}
					return 1;
					//printf("right: ranges[i] = %f\n",laserscan.ranges[i]);
				}
			}
		}
	}

	//printf("rPlantData: %d\n",rPlantData);
	//if(rPlantData == 1){
	//	return 1;
	//}
	//else if(rPlantData == 3){
	//	return 3;
	//}
	return 0;
}

int MaizeMapping::getNavState(){
	if(navStateData == 1){
		return 1; //end of row	
	}
	return 0;
}

void MaizeMapping::loop()
{
	ros::spinOnce();

	generateMap();

	//printf("scan_time = %lf\n",laserscan.scan_time);
	if(!laserscan.header.stamp.is_zero())
		ros::Duration(laserscan.scan_time).sleep();
	else
		ros::Duration(0.01).sleep(); //sleep 10ms
}

void MaizeMapping::generateMap()
{
	//Track the position of the robot
	tf::Pose pose_start, pose_current, pose_diff, pose_bor, pose_eor, pose_dor;
	//Use a OpenCV Matrix to store the map image 
	Mat matMap,matPlants, matRed;
	int imageSizeX = (rowCount - 1)*(rowSpacing * mapResolution) + 2*mapPadding;
	int imageSizeY = rowLength * mapResolution + 2*mapPadding;
	bool breakwhileloop = false;
	//Keep track of where to draw the next plant and robot		
	Point robotStartPt, robotCurrPt, lPlantPt, rPlantPt;
	int r2lsign = 1;
	int endOfRowYPos = imageSizeY-mapPadding;
	int reverseDirection = 1;

	matMap = Mat(imageSizeY, imageSizeX, CV_8UC3, Scalar(190,190,190));
	matPlants = Mat(imageSizeY, imageSizeX, CV_8UC3, Scalar(190,190,190));
	matRed = Mat(imageSizeY, imageSizeX, CV_8UC3, Scalar(190,190,190));
	namedWindow("Maize Map",WINDOW_NORMAL);
	for(int currentRow = 0; currentRow < rowCount-1; currentRow++){
		breakwhileloop = false;
		reverseDirection = reverseDirection*-1;
		while(navStateData != 9 && ros::ok()){
			ros::spinOnce();
			//printf("waiting for row nav\n");
			//do nothing
		}
		tf::poseMsgToTF(odom.pose.pose, pose_start); // get the starting position with respect to odom
		tf::poseMsgToTF(odom.pose.pose, pose_bor); // get the starting position with respect to odom
		printf("start of row: %d\n",currentRow+1);
		navStateData = 0;
		//Define first plant locations
		//The Origin point (0,0) of OpenCV Mat is in the top-left corner
		if(pathL2R == true){
			r2lsign = 1;
			//This puts the starting plants on the bottom left with respect to the resulting image
			robotStartPt = Point(mapPadding+rowSpacing/2*mapResolution+currentRow*rowSpacing*mapResolution,endOfRowYPos);
			/**if(currentRow == 0){
				robotStartPt = Point(mapPadding+rowSpacing/2*mapResolution+currentRow*rowSpacing*mapResolution,endOfRowYPos);
			}
			else{
				pose_dor = pose_start.inverse() * pose_current; // get current position with respect to starting position
				robotStartPt = Point(mapPadding+rowSpacing/2*mapResolution+currentRow*rowSpacing*mapResolution,endOfRowYPos+pose_dor.getOrigin().x()*mapResolution);
			}**/
		}
		else{
			r2lsign = -1;
			// '' on the bottom right ''
			robotStartPt = Point(imageSizeX-mapPadding-rowSpacing/2*mapResolution-currentRow*rowSpacing*mapResolution,endOfRowYPos);
			/**if(currentRow == 0){
				robotStartPt = Point(imageSizeX-mapPadding-rowSpacing/2*mapResolution-currentRow*rowSpacing*mapResolution,endOfRowYPos);
			}
			else{
				pose_dor = pose_start.inverse() * pose_current; // get current position with respect to starting position
				robotStartPt = Point(imageSizeX-mapPadding-rowSpacing/2*mapResolution-currentRow*rowSpacing*mapResolution,endOfRowYPos+pose_dor.getOrigin().x()*mapResolution);
			}**/
		}
		//printf("y start position: %d\n",robotStartPt.y);
		robotCurrPt = robotStartPt;
		while(ros::ok() && breakwhileloop == false){
			ros::spinOnce(); //need to spin to get new data from callbackspose_diff.getOrigin().x()*mapResolution

			//paint robot's path on map
			tf::poseMsgToTF(odom.pose.pose, pose_current); // get current position with respect to odom
			pose_diff = pose_start.inverse() * pose_current; // get current position with respect to starting position
			robotCurrPt = robotStartPt + Point(0,reverseDirection*pose_diff.getOrigin().x()*mapResolution);
			lPlantPt = robotCurrPt + Point(reverseDirection*mapResolution*rowSpacing/2,0);
			rPlantPt = robotCurrPt + Point(-reverseDirection*mapResolution*rowSpacing/2,0);
			circle(matMap, robotCurrPt, mapResolution*rowSpacing/2, Scalar(255,255,255), -1, 4, 0);

			if(reverseDirection+r2lsign == 0){
				if(detectLeftPlant()==1){
					circle(matMap, lPlantPt, 6, Scalar(0,0,0), -1, 4, 0);
					circle(matPlants, lPlantPt, 6, Scalar(0,0,0), -1, 4, 0);
				}
				else if(detectLeftPlant()==3){
					circle(matMap, lPlantPt, 6, Scalar(0,0,255), -1, 4, 0);
					circle(matRed, lPlantPt, 6, Scalar(0,0,255), -1, 4, 0);
				}
			}
			if(reverseDirection+r2lsign == -2 || reverseDirection+r2lsign == 2){
				if(detectRightPlant()==1){
					circle(matMap, rPlantPt, 6, Scalar(0,0,0), -1, 4, 0);
					circle(matPlants, rPlantPt, 6, Scalar(0,0,0), -1, 4, 0);
				}
				else if(detectRightPlant()==3){
					circle(matMap, rPlantPt, 6, Scalar(0,0,255), -1, 4, 0);
					circle(matRed, rPlantPt, 6, Scalar(0,0,255), -1, 4, 0);
				}
			}
			/**if(navStateData == ){
				navStateData = 0;
				circle(matMap, robotCurrPt, mapResolution*rowSpacing/2+10, Scalar(255,0,0), 1, 4, 0);
				printf("end of row\n");
			}**/
			if(navStateData == 1){
				navStateData = 0;
				//circle(matMap, robotCurrPt, 40, Scalar(255,0,0), 1, 4, 0);
				printf("turning\n");
				endOfRowYPos = robotCurrPt.y;
				tf::poseMsgToTF(odom.pose.pose, pose_eor); // get the starting position with respect to odom
				breakwhileloop = true;
			}
			//Display Map in window
			imshow("Maize Map", matMap);
			//cvResizeWindow("Maize Map", imageSizeY * 2, imageSizeX * 2);
		}
	}
	//cv_bridge::CvImage img_message;
	//header.stamp = ros::Time::now();
	//header.frame_id = "main_map";
	//header.seq = sequence++;
	//img_message.header = header;
	//img_message.encoding = sensor_msgs::image_encodings::BGR8;
	//img_message.image = matMap;
	//maize_map_pub_.publish(img_message.toImageMsg());
	Vec3b grey = (190,190,190);
	Vec3b black = (0,0,0);
	Vec3b white = (255,255,255);
	for(int i = 0; i<imageSizeX;i++){
		for(int j = 0; j<imageSizeY;j++){
			if(matRed.at<Vec3b>(Point(i,j))[0] == 190){
				printf("matplants\n");
				if(matPlants.at<Vec3b>(Point(i,j))[0] == 0){
					matRed.at<Vec3b>(Point(i,j)) = matPlants.at<Vec3b>(Point(i,j));
				}
				else{
					if(matMap.at<Vec3b>(Point(i,j))[0] == 255){
						matRed.at<Vec3b>(Point(i,j)) = matMap.at<Vec3b>(Point(i,j));
					}
				}
			}
		}
	}

	imwrite("/home/teamfieldrobot/MaizeMap.tiff",matRed);
	printf("printed MaizeMap.tiff");

	matMap.release();
	matPlants.release();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "maizeMapping");
  ros::NodeHandle n;

  MaizeMapping map;
  map.init();
  map.loop();

  ros::spin(); //This keeps the node on
  return 0;
}
