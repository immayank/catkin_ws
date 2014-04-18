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


namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

#define MAX_COUNT 500
image_transport::Publisher facepub;
char rawWindow[] = "Raw Video";
char opticalFlowWindow[] = "Optical Flow Window";
char imageFileName[32];
long imageIndex = 0;
char keyPressed;
Mat prev_left_gray;
vector<Point2f> points1;
vector<Point2f> points2;

Point2f diff;
vector<uchar> status;
vector<float> err;

RNG rng(12345);
Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
bool needToInit = true;

int i, k;
TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
Size subPixWinSize(10, 10), winSize(31, 31);
double angle;


Mat left_opflow(Mat left_gray, Mat prev_left_gray){
		Mat opticalFlow;
		left_gray.copyTo(opticalFlow);
		if (needToInit)
		{
			goodFeaturesToTrack(left_gray, points1, MAX_COUNT, 0.01, 100, Mat(), 6, 0, 0.04);
			needToInit = false;
		}
		else if (!points2.empty())
		{
//			cout << "\n\n\nCalculating  calcOpticalFlowPyrLK\n\n\n\n\n";
			calcOpticalFlowPyrLK(prev_left_gray, left_gray, points2, points1, status, err, winSize, 3, termcrit, 0, 0.001);

			for (i = k = 0; i < points2.size(); i++)
			{
				if ((points1[i].x - points2[i].x) > 0)
				{
					line(left_gray, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
					circle(left_gray, points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);

					line(opticalFlow, points1[i], points2[i], Scalar(0, 0, 255), 1, 1, 0);
					circle(opticalFlow, points1[i], 1, Scalar(255, 0, 0), 1, 1, 0);
				}
				else
				{
					line(left_gray, points1[i], points2[i], Scalar(0, 255, 0), 1, 1, 0);
					circle(left_gray, points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);

					line(opticalFlow, points1[i], points2[i], Scalar(0, 255, 0), 1, 1, 0);
					circle(opticalFlow, points1[i], 1, Scalar(255, 0, 0), 1, 1, 0);
				}
				points1[k++] = points1[i];
			}

			goodFeaturesToTrack(left_gray, points1, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);

		}
		std::swap(points2, points1);
		points1.clear();
	return (opticalFlow);
}

void imageCallback(const ImageConstPtr& imagel,const ImageConstPtr& imager)
{

	
	cv_bridge::CvImagePtr origl_img;
	cv_bridge::CvImagePtr origr_img;
	origl_img = cv_bridge::toCvCopy(imagel, enc::BGR8);
	origr_img = cv_bridge::toCvCopy(imager, enc::BGR8);
	
	cv::Mat left_img = origl_img->image;
	cv::Mat right_img = origr_img->image;
	Mat left_gray,right_gray,disp,disp8_sgbm,disp8_sbm;
	cvtColor(left_img, left_gray, CV_BGR2GRAY);
	cvtColor(right_img, right_gray, CV_BGR2GRAY);
	

// Stereo BM
	StereoBM sbm;
	sbm.state->SADWindowSize = 9;
	sbm.state->numberOfDisparities = 112;
	sbm.state->preFilterSize = 5;
	sbm.state->preFilterCap = 61;
	sbm.state->minDisparity = -39;
	sbm.state->textureThreshold = 507;
	sbm.state->uniquenessRatio = 0;
	sbm.state->speckleWindowSize = 0;
	sbm.state->speckleRange = 8;
	sbm.state->disp12MaxDiff = 1;
	sbm(left_gray, right_gray, disp);
	normalize(disp, disp8_sbm, 0, 255, CV_MINMAX, CV_8U);

// Stereo SGBM	
	StereoSGBM sgbm;
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 192;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = -64;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;
	sgbm(left_gray, right_gray, disp);

	normalize(disp, disp8_sgbm, 0, 255, CV_MINMAX, CV_8U);
	
	Mat visual_l = left_opflow(left_gray,prev_left_gray);
	left_gray.copyTo(prev_left_gray);
	
	cv::imshow("left_img",left_img);	
	cv::imshow("right_img",right_img);
	cv::imshow("Disparitysbm",disp8_sbm);
	cv::imshow("Disparitysgbm",disp8_sgbm);
	imshow("View",visual_l);
//	printf("Code works \n");
	cvWaitKey(2);
	return;
	
}



void initialize_vars()
	{
	
}


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "stereo");
	ros::NodeHandle nh;
	initialize_vars();
	image_transport::ImageTransport *it   = new image_transport::ImageTransport(nh);

// GsCam
	/*message_filters::Subscriber<Image> imagel_sub(nh, "minoru/left/camera/image_raw", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "minoru/right/camera/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"minoru/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"minoru/right/camera_info", 1);*/

// MIT Database
	message_filters::Subscriber<Image> imagel_sub(nh, "/wide_stereo/left/image_rect", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "/wide_stereo/right/image_rect", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"/wide_stereo/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"/wide_stereo/right/camera_info", 1);

// Vslam
	/*message_filters::Subscriber<Image> imagel_sub(nh, "/narrow_stereo/left/image_rect", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "/narrow_stereo/right/image_rect", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"/narrow_stereo/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"/narrow_stereo/right/camera_info", 1);*/

// blue bag
	/*message_filters::Subscriber<Image> imagel_sub(nh, "/bumblebee/left/image_rect_resized", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "/bumblebee/right/image_rect_resized", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"bumblebee/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"bumblebee/right/camera_info", 1);*/
	
// Rotating detergent
	/*message_filters::Subscriber<Image> imagel_sub(nh, "/narrow_stereo_textured/left/image_raw", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "/narrow_stereo_textured/right/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"/narrow_stereo_textured/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"/narrow_stereo_textured/right/camera_info", 1);*/

	typedef sync_policies::ApproximateTime<Image,Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imagel_sub,imager_sub);
	sync.registerCallback(boost::bind(&imageCallback, _1, _2));
//	image_transport::Subscriber imagemaskSub = it->subscribe   ("/riverdetector/visualization" , 1, maskimageCb);
	while (ros::ok())
	{
	

//	ROS_INFO("I published something!");

	ros::spinOnce();
}
  return 0;
}
