
#include<vector>
#include<math.h>
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
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <stdio.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
float nndrRatio = 0.70f;
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
int der_x=1,der_y=1,kernel=1,value=1500,ratio=0.65f;

Mat left_old,right_old;
int iteration=1;

void imageCallback(const ImageConstPtr& imagel,const ImageConstPtr& imager)
{

	cv_bridge::CvImagePtr origl_img;
	cv_bridge::CvImagePtr origr_img;
	origl_img = cv_bridge::toCvCopy(imagel, enc::BGR8);
	origr_img = cv_bridge::toCvCopy(imager, enc::BGR8);
	
	cv::Mat left_new = origl_img->image;
	cv::Mat right_new = origr_img->image;
	Mat new_gray,disp,disp8_sgbm,disp8_sbm;
	cvtColor(left_new, left_new, CV_BGR2GRAY);
	cvtColor(right_new, right_new, CV_BGR2GRAY);
	
	if (iteration>2){
	     

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//First find points between left(t) and left(t-1)//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/
/*
    ORB orb1;
    // Feature detection
    Mat descriptor1,descriptor2;
    vector<KeyPoint> keypoints1;
    vector<KeyPoint> keypoints2;
    orb1(left_new,Mat(),keypoints1,descriptor1);
    orb1(left_old,Mat(),keypoints2,descriptor2);
    
    
    // corresponded points
	vector< vector< DMatch >  > matches1;
	vector< vector< DMatch >  > matches2;
 
    // L2 distance based matching. Brute Force Matching
    BFMatcher matcher1(NORM_L1); 
 
    // display of corresponding points
    matcher1.knnMatch( descriptor1, descriptor2, matches1 ,2);
    matcher1.knnMatch( descriptor2, descriptor1, matches2 ,2);
     
   
    // Symmetry test
    vector<DMatch> symMatches;

	for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin();matchIterator1!= matches1.end(); ++matchIterator1) {
		// ignore deleted matches
		if (matchIterator1->size() < 2)
		    continue;
		// for all matches image 2 -> image 1
		for (std::vector<std::vector<cv::DMatch> >::
		     const_iterator matchIterator2= matches2.begin();
		     matchIterator2!= matches2.end();
		     ++matchIterator2) {
		    // ignore deleted matches
		    if (matchIterator2->size() < 2)
		        continue;
		    // Match symmetry test
		    if ((*matchIterator1)[0].queryIdx ==
		        (*matchIterator2)[0].trainIdx &&
		        (*matchIterator2)[0].queryIdx ==
		        (*matchIterator1)[0].trainIdx) {
		        // add symmetrical match
		        symMatches.push_back(cv::DMatch((*matchIterator1)[0].queryIdx,
		                                        (*matchIterator1)[0].trainIdx,
		                                        (*matchIterator1)[0].distance));
		        break; // next match in image 1 -> image 2
		    }
		}
	    }

   	std::vector<cv::DMatch> outMatches;
	std::vector<cv::Point2f> points1;	
	std::vector<cv::Point2f> points2;
    cv::Mat fundamental;
    for (std::vector<cv::DMatch>::
         const_iterator it= symMatches.begin();
         it!= symMatches.end(); ++it) {
        // Get the position of left keypoints
        float x= keypoints1[it->queryIdx].pt.x;
        float y= keypoints1[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= keypoints2[it->trainIdx].pt.x;
        y= keypoints2[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers(points1.size(),0);
    if (points1.size()>0&&points2.size()>0){
        cv::Mat fundamental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), // matching points
                                                    inliers,       // match status (inlier or outlier)
                                                    CV_FM_RANSAC, // RANSAC method
                                                    3,      // distance to epipolar line
                                                    0.99); // confidence probability
	bool refineF = true;
        // extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator
        itIn= inliers.begin();
        std::vector<cv::DMatch>::const_iterator
        itM= symMatches.begin();
        // for all matches
        for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
            if (*itIn) { // it is a valid match
                outMatches.push_back(*itM);
            }
        }
        if (refineF) {
            // The F matrix will be recomputed with
            // all accepted matches
            // Convert keypoints into Point2f
            // for final F computation
            points1.clear();
            points2.clear();
            for (std::vector<cv::DMatch>::
                 const_iterator it= outMatches.begin();
                 it!= outMatches.end(); ++it) {
                // Get the position of left keypoints
                float x= keypoints1[it->queryIdx].pt.x;
                float y= keypoints1[it->queryIdx].pt.y;
                points1.push_back(cv::Point2f(x,y));
                // Get the position of right keypoints
                x= keypoints2[it->trainIdx].pt.x;
                y= keypoints2[it->trainIdx].pt.y;
                points2.push_back(cv::Point2f(x,y));
            }
            // Compute 8-point F from all accepted matches
            if (points1.size()>0&&points2.size()>0){
                fundamental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), // matches
                                                    CV_FM_8POINT); // 8-point method
		cout << "Fundamental = "<< endl << " "  <<fundamental << endl << endl;
            }
        }
    }
  Mat result1;	
  drawMatches( left_new, keypoints1, left_old, keypoints2, outMatches, result1 );*/


/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// We now correspondence between left(t-1) and right (t-1)//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/
	 ORB orb2;
    // Feature detection
    Mat descriptor3,descriptor4;
    vector<KeyPoint> keypoints3;
    vector<KeyPoint> keypoints4;
    orb2(left_old,Mat(),keypoints3,descriptor3);
    orb2(right_old,Mat(),keypoints4,descriptor4);
    
    
    // corresponded points
	vector< vector< DMatch >  > matches3;
	vector< vector< DMatch >  > matches4;
	BFMatcher matcher2(NORM_L1);
 
//
	matcher2.knnMatch( descriptor3, descriptor4, matches3 ,2);
	matcher2.knnMatch( descriptor4, descriptor3, matches4 ,2);

    // Symmetry test
    vector<DMatch> symMatches2;

	for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator3= matches3.begin();matchIterator3!= matches3.end(); ++matchIterator3) {
		// ignore deleted matches
		if (matchIterator3->size() < 2)
		    continue;
		// for all matches image 2 -> image 1
		for (std::vector<std::vector<cv::DMatch> >::
		     const_iterator matchIterator4= matches4.begin();
		     matchIterator4!= matches4.end();
		     ++matchIterator4) {
		    // ignore deleted matches
		    if (matchIterator4->size() < 2)
		        continue;
		    // Match symmetry test
		    if ((*matchIterator3)[0].queryIdx ==
		        (*matchIterator4)[0].trainIdx &&
		        (*matchIterator4)[0].queryIdx ==
		        (*matchIterator3)[0].trainIdx) {
		        // add symmetrical match
		        symMatches2.push_back(cv::DMatch((*matchIterator3)[0].queryIdx,
		                                        (*matchIterator3)[0].trainIdx,
		                                        (*matchIterator3)[0].distance));
		        break; // next match in image 1 -> image 2
		    }
		}
	    }

	std::vector<cv::DMatch> outMatches2;
	std::vector<cv::Point2f> points3;	
	std::vector<cv::Point2f> points4;
    cv::Mat fundamental2;
    for (std::vector<cv::DMatch>::
         const_iterator it= symMatches2.begin();
         it!= symMatches2.end(); ++it) {
        // Get the position of left keypoints
        float x= keypoints3[it->queryIdx].pt.x;
        float y= keypoints4[it->queryIdx].pt.y;
        points3.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= keypoints3[it->trainIdx].pt.x;
        y= keypoints4[it->trainIdx].pt.y;
        points4.push_back(cv::Point2f(x,y));
    }
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers2(points3.size(),0);
    if (points3.size()>0&&points4.size()>0){
        cv::Mat fundamental2= cv::findFundamentalMat(cv::Mat(points3),cv::Mat(points4), // matching points
                                                    inliers2,       // match status (inlier or outlier)
                                                    CV_FM_RANSAC, // RANSAC method
                                                    1,      // distance to epipolar line
                                                    0.99); // confidence probability
	
        // extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator
        itIn= inliers2.begin();
        std::vector<cv::DMatch>::const_iterator
        itM= symMatches2.begin();
	bool refineF = true;
        // for all matches
        for ( ;itIn!= inliers2.end(); ++itIn, ++itM) {
            if (*itIn) { // it is a valid match
                outMatches2.push_back(*itM);
            }
        }
        if (refineF) {
            // The F matrix will be recomputed with
            // all accepted matches
            // Convert keypoints into Point2f
            // for final F computation
            points3.clear();
            points4.clear();
            for (std::vector<cv::DMatch>::
                 const_iterator it= outMatches2.begin();
                 it!= outMatches2.end(); ++it) {
                // Get the position of left keypoints
                float x= keypoints3[it->queryIdx].pt.x;
                float y= keypoints3[it->queryIdx].pt.y;
                points3.push_back(cv::Point2f(x,y));
                // Get the position of right keypoints
                x= keypoints4[it->trainIdx].pt.x;
                y= keypoints4[it->trainIdx].pt.y;
                points4.push_back(cv::Point2f(x,y));
            }
            // Compute 8-point F from all accepted matches
            if (points3.size()>0&&points4.size()>0){
                fundamental2= cv::findFundamentalMat(cv::Mat(points3),cv::Mat(points4), // matches
                                                    CV_FM_8POINT); // 8-point method
//		cout << "Fundamental = "<< endl << " "  <<fundamental << endl << endl;
            }
        }
    }
  Mat result2;	
  drawMatches( left_old, keypoints3, right_old, keypoints4, outMatches2, result2 );




//   imshow("Left(t) and Left(t-1)", result1);
    imshow("Left(t-1) and Right(t-1)", result2);
//    imshow("After Ransac", result3);
    waitKey(10); //press any key to quit
    
    }
    iteration++;
    left_old=left_new;
    right_old=right_new;
    waitKey(10);
    
}	



int main(int argc, char** argv)
{	
	ros::init(argc, argv, "feature_extract");
	ros::NodeHandle nh;
	
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
