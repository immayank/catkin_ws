#include <ros/ros.h>
#include <iomanip>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <vector>


using namespace std;
using namespace cv;
using namespace sensor_msgs;
int i=0,j=0;
Mat path_b(300, 400, CV_8UC3, Scalar::all(255));
Mat path_st(300, 400, CV_8UC3, Scalar::all(255));

	
void callback1(const nav_msgs::Odometry base_odom)
{
	/*vector<double> x;
	vector<double> y;
	vector<double> z;

	double vx=0.0;
	double vy=0.0;
	double vz=0.0;

	x.push_back(base_odom.pose.pose.position.x);
	y.push_back(base_odom.pose.pose.position.y);
	z.push_back(base_odom.pose.pose.position.z);
	//i=x.size();*/
	double posx_b = -8*base_odom.pose.pose.position.x;
	double posy_b = -8*base_odom.pose.pose.position.y;	
	
	Point position(posx_b,posy_b); 
	
	circle(path_b, (position), 1, Scalar(255, 0, 0), -1, 8, 0);
	imshow("View_b",path_b);
//	cout << position.x <<" " << position.y<<"\n";	
	cvWaitKey(2);
	
	//cout <<x.at(x.size()) << " "<<y.at(y.size())<<" "<<z.at(z.size())<<" "<<"\n";	
	return;

}

void callback2(const nav_msgs::Odometry cam_odom)
{
	double x = cam_odom.pose.pose.position.x;
	double y = cam_odom.pose.pose.position.y;
	double z = cam_odom.pose.pose.position.z;
	
	if (x==0 && y==0 && z==0){}
	else{
		ofstream outfile;
		outfile.open("/home/mayank/path.txt", std::ios_base::app);
		outfile <<cam_odom.pose.pose.position.x<< "\t"<<cam_odom.pose.pose.position.y<<"\t"<<cam_odom.pose.pose.position.z<<"\n";
	}
	double posx_st = 6*cam_odom.pose.pose.position.x+150;
	double posy_st = 6*cam_odom.pose.pose.position.y+200;
	double posz_st = cam_odom.pose.pose.position.z;	
	
	Point position1(posx_st,posy_st); 
	
	circle(path_st, (position1), 1, Scalar(255, 0, 0), -1, 8, 0);
	//~ imshow("View_st",path_st);
	cout << cam_odom.pose.pose.position.x <<" " << cam_odom.pose.pose.position.y<<" "<<posz_st << "\n";	
	//~ cvWaitKey(2);
	
	//cout <<x.at(x.size()) << " "<<y.at(y.size())<<" "<<z.at(z.size())<<" "<<"\n";	
	return;
}




int main(int argc, char** argv)
{	
	ros::init(argc, argv, "path_map");
	ros::NodeHandle nh;
	ros::Subscriber base_odom = nh.subscribe("/base_odometry/odom", 1, callback1);
	ros::Subscriber cam_odom = nh.subscribe("/stereo_odometer/odometry", 10, callback2);

	while (ros::ok())
	{
	

//	ROS_INFO("I published something!");

	ros::spinOnce();
}
  return 0;
}
