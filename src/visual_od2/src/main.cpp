#include <vector>
#include <math.h>
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
#include <image_geometry/stereo_camera_model.h>
#include "viso_stereo.h"
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ctime>
#include <sys/stat.h>
#include <sstream>


// Define Namespace
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace image_geometry;


struct pointmatch{
	public:
		Point2f point1;
		Point2f point2;
		Point2f point3;
		Point2f point4;
		
		//~ pointmatch () { };
		//~ ~pointmatch ();
		//~ 
};



// Define Global Variables
float nndrRatio = 0.70f;
int der_x=1,der_y=1,kernel=1,value=1500,ratio=0.65f;
Mat left_new,right_new;
Mat left_old,right_old;
int iteration=1;
//Matx44d pose = Matx44d::eye();
Matrix pose = Matrix::eye(4);
Matrix reference_motion = Matrix::eye(4);
double ref_frame_motion_threshold = 5;
int ref_frame_inlier_threshold = 100;
int ref_frame_change_method=0;
bool change_reference_frame = false;
int iter123=1;
tf::Transform integrated_pose;
char* dt;
string filename1;
string filename2;
  
VisualOdometryStereo::parameters getCameraparams(StereoCameraModel model){
	VisualOdometryStereo::parameters param;
	param.calib.cu=model.left().cx();
	param.calib.cv=model.left().cy();
	param.calib.f=model.left().fx();
	param.base = model.baseline();
	//~ cout << "CU =" << param.cu<<endl;
	return param;
}
 
   
/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Callback funtion at the time of data received//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

void imageCallback(const ImageConstPtr& imagel,const ImageConstPtr& imager, const CameraInfoConstPtr& l_info_msg,const CameraInfoConstPtr& r_info_msg)
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
	StereoCameraModel model;
	model.fromCameraInfo(*l_info_msg, *r_info_msg);
	VisualOdometryStereo::parameters params = getCameraparams(model);
	
	if (iteration>2){
	ORB orb1(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb2(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb3(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb4(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb5(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb6(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb7(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	ORB orb8(4000, 1.2f, 8, 31, 0, 2, 0, 31);
	//~ ORB orb1;
	//~ ORB orb2;
	//~ ORB orb3;
	//~ ORB orb4;
	
	//~ SIFT orb1;
	//~ SIFT orb2;
	//~ SIFT orb3;
	//~ SIFT orb4;
/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//First find points between left(t) and left(t-1)//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

    
    // Feature detection
    Mat descriptor1,descriptor2;
    vector<KeyPoint> keypoints1;
    vector<KeyPoint> keypoints2;
    orb1(left_new,Mat(),keypoints1,descriptor1);
    orb2(left_old,Mat(),keypoints2,descriptor2);
    
    
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
                                                    0.999); // confidence probability
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
//		cout << "Fundamental = "<< endl << " "  <<fundamental << endl << endl;
            }
        }
    }
  Mat result1;	
  drawMatches( left_new, keypoints1, left_old, keypoints2, outMatches, result1 );


/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// We now correspondence between left(t-1) and right (t-1)//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

	 
    // Feature detection
    Mat descriptor3,descriptor4;
    vector<KeyPoint> keypoints3;
    vector<KeyPoint> keypoints4;
    orb3(left_old,Mat(),keypoints3,descriptor3);
    orb4(right_old,Mat(),keypoints4,descriptor4);
    
    
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
        float y= keypoints3[it->queryIdx].pt.y;
        points3.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= keypoints4[it->trainIdx].pt.x;
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
                                                    0.999); // confidence probability
	
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

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// We now correspondence between right(t-1) and right (t)//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

    // Feature detection
    Mat descriptor5,descriptor6;
    vector<KeyPoint> keypoints5;
    vector<KeyPoint> keypoints6;
    orb5(right_old,Mat(),keypoints5,descriptor5);
    orb6(right_new,Mat(),keypoints6,descriptor6);
    
    
    // corresponded points
	vector< vector< DMatch >  > matches5;
	vector< vector< DMatch >  > matches6;
	BFMatcher matcher3(NORM_L1);
 
//
	matcher3.knnMatch( descriptor5, descriptor6, matches5 ,2);
	matcher3.knnMatch( descriptor6, descriptor5, matches6 ,2);

    // Symmetry test
    vector<DMatch> symMatches3;

	for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator5= matches5.begin();matchIterator5!= matches5.end(); ++matchIterator5) {
		// ignore deleted matches
		if (matchIterator5->size() < 2)
		    continue;
		// for all matches image 2 -> image 1
		for (std::vector<std::vector<cv::DMatch> >::
		     const_iterator matchIterator6= matches6.begin();
		     matchIterator6!= matches6.end();
		     ++matchIterator6) {
		    // ignore deleted matches
		    if (matchIterator6->size() < 2)
		        continue;
		    // Match symmetry test
		    if ((*matchIterator5)[0].queryIdx ==
		        (*matchIterator6)[0].trainIdx &&
		        (*matchIterator6)[0].queryIdx ==
		        (*matchIterator5)[0].trainIdx) {
		        // add symmetrical match
		        symMatches3.push_back(cv::DMatch((*matchIterator5)[0].queryIdx,
		                                        (*matchIterator5)[0].trainIdx,
		                                        (*matchIterator5)[0].distance));
		        break; // next match in image 1 -> image 2
		    }
		}
	    }

	std::vector<cv::DMatch> outMatches3;
	std::vector<cv::Point2f> points5;	
	std::vector<cv::Point2f> points6;
    cv::Mat fundamental3;
    for (std::vector<cv::DMatch>::
         const_iterator it= symMatches3.begin();
         it!= symMatches3.end(); ++it) {
        // Get the position of left keypoints
        float x= keypoints5[it->queryIdx].pt.x;
        float y= keypoints5[it->queryIdx].pt.y;
        points5.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= keypoints6[it->trainIdx].pt.x;
        y= keypoints6[it->trainIdx].pt.y;
        points6.push_back(cv::Point2f(x,y));
    }
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers3(points5.size(),0);
    if (points5.size()>0&&points6.size()>0){
        cv::Mat fundamental3= cv::findFundamentalMat(cv::Mat(points5),cv::Mat(points6), // matching points
                                                    inliers3,       // match status (inlier or outlier)
                                                    CV_FM_RANSAC, // RANSAC method
                                                    1,      // distance to epipolar line
                                                    0.999); // confidence probability
	
        // extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator
        itIn= inliers3.begin();
        std::vector<cv::DMatch>::const_iterator
        itM= symMatches3.begin();
	bool refineF = true;
        // for all matches
        for ( ;itIn!= inliers3.end(); ++itIn, ++itM) {
            if (*itIn) { // it is a valid match
                outMatches3.push_back(*itM);
            }
        }
        if (refineF) {
            // The F matrix will be recomputed with
            // all accepted matches
            // Convert keypoints into Point2f
            // for final F computation
            points5.clear();
            points6.clear();
            for (std::vector<cv::DMatch>::
                 const_iterator it= outMatches3.begin();
                 it!= outMatches3.end(); ++it) {
                // Get the position of left keypoints
                float x= keypoints5[it->queryIdx].pt.x;
                float y= keypoints5[it->queryIdx].pt.y;
                points5.push_back(cv::Point2f(x,y));
                // Get the position of right keypoints
                x= keypoints6[it->trainIdx].pt.x;
                y= keypoints6[it->trainIdx].pt.y;
                points6.push_back(cv::Point2f(x,y));
            }
            // Compute 8-point F from all accepted matches
            if (points5.size()>0&&points6.size()>0){
                fundamental3= cv::findFundamentalMat(cv::Mat(points5),cv::Mat(points6), // matches
                                                    CV_FM_8POINT); // 8-point method
//		cout << "Fundamental = "<< endl << " "  <<fundamental << endl << endl;
            }
        }
    }
  Mat result3;	
  drawMatches( right_old, keypoints5, right_new, keypoints6, outMatches3, result3 );

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// We now correspondence between right(t) and left (t)//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/


     // Feature detection
    Mat descriptor7,descriptor8;
    vector<KeyPoint> keypoints7;
    vector<KeyPoint> keypoints8;
    orb7(right_new,Mat(),keypoints7,descriptor7);
    orb8(left_new,Mat(),keypoints8,descriptor8);
    
    
    // corresponded points
	vector< vector< DMatch >  > matches7;
	vector< vector< DMatch >  > matches8;
	BFMatcher matcher4(NORM_L1);
 
//
	matcher4.knnMatch( descriptor7, descriptor8, matches7 ,2);
	matcher4.knnMatch( descriptor8, descriptor7, matches8,2);

    // Symmetry test
    vector<DMatch> symMatches4;

	for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator7= matches7.begin();matchIterator7!= matches7.end(); ++matchIterator7) {
		// ignore deleted matches
		if (matchIterator7->size() < 2)
		    continue;
		// for all matches image 2 -> image 1
		for (std::vector<std::vector<cv::DMatch> >::
		     const_iterator matchIterator8= matches8.begin();
		     matchIterator8!= matches8.end();
		     ++matchIterator8) {
		    // ignore deleted matches
		    if (matchIterator8->size() < 2)
		        continue;
		    // Match symmetry test
		    if ((*matchIterator7)[0].queryIdx ==
		        (*matchIterator8)[0].trainIdx &&
		        (*matchIterator8)[0].queryIdx ==
		        (*matchIterator7)[0].trainIdx) {
		        // add symmetrical match
		        symMatches4.push_back(cv::DMatch((*matchIterator7)[0].queryIdx,
		                                        (*matchIterator7)[0].trainIdx,
		                                        (*matchIterator7)[0].distance));
		        break; // next match in image 1 -> image 2
		    }
		}
	   }

	std::vector<cv::DMatch> outMatches4;
	std::vector<cv::Point2f> points7;	
	std::vector<cv::Point2f> points8;
    cv::Mat fundamental4;
    for (std::vector<cv::DMatch>::
         const_iterator it= symMatches4.begin();
         it!= symMatches4.end(); ++it) {
        // Get the position of left keypoints
        float x= keypoints7[it->queryIdx].pt.x;
        float y= keypoints7[it->queryIdx].pt.y;
        points7.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= keypoints8[it->trainIdx].pt.x;
        y= keypoints8[it->trainIdx].pt.y;
        points8.push_back(cv::Point2f(x,y));
    }
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers4(points7.size(),0);
    if (points7.size()>0&&points8.size()>0){
        cv::Mat fundamental4= cv::findFundamentalMat(cv::Mat(points7),cv::Mat(points8), // matching points
                                                    inliers4,       // match status (inlier or outlier)
                                                    CV_FM_RANSAC, // RANSAC method
                                                    1,      // distance to epipolar line
                                                    0.999); // confidence probability
	
        // extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator
        itIn= inliers4.begin();
        std::vector<cv::DMatch>::const_iterator
        itM= symMatches4.begin();
		bool refineF = true;
        // for all matches
        for ( ;itIn!= inliers4.end(); ++itIn, ++itM) {
            if (*itIn) { // it is a valid match
                outMatches4.push_back(*itM);
            }
        }
        if (refineF) {
            // The F matrix will be recomputed with
            // all accepted matches
            // Convert keypoints into Point2f
            // for final F computation
            points7.clear();
            points8.clear();
            for (std::vector<cv::DMatch>::
                 const_iterator it= outMatches4.begin();
                 it!= outMatches4.end(); ++it) {
                // Get the position of left keypoints
                float x= keypoints7[it->queryIdx].pt.x;
                float y= keypoints7[it->queryIdx].pt.y;
                points7.push_back(cv::Point2f(x,y));
                // Get the position of right keypoints
                x= keypoints8[it->trainIdx].pt.x;
                y= keypoints8[it->trainIdx].pt.y;
                points8.push_back(cv::Point2f(x,y));
            }
            // Compute 8-point F from all accepted matches
            if (points7.size()>0&&points8.size()>0){
                fundamental4= cv::findFundamentalMat(cv::Mat(points7),cv::Mat(points8), // matches
                                                    CV_FM_8POINT); // 8-point method
//		cout << "Fundamental = "<< endl << " "  <<fundamental << endl << endl;
            }
        }
    }
  Mat result4;	
  drawMatches( right_new, keypoints7, left_new, keypoints8, outMatches4, result4 );

//

   resize(result1, result1, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
   resize(result2, result2, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
   resize(result3, result3, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
   resize(result4, result4, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
	

//    imshow("Left(t) and Left(t-1)", result1);
//    imshow("Left(t-1) and Right(t-1)", result2);
//    imshow("Right(t-1) and Right(t)", result3);
//    imshow("Right(t) and Left(t)", result4);
    Mat im3((result1.size()).height*2, (result1.size()).width*2, CV_8UC3);
    Mat left_top(im3, Rect(0, 0, (result1.size()).width, (result1.size()).height));
    Mat right_top(im3, Rect((result1.size()).width, 0, (result1.size()).width, (result1.size()).height));
    Mat left_bottom(im3, Rect(0, (result1.size()).height, (result1.size()).width, (result1.size()).height));
    Mat right_bottom(im3, Rect((result1.size()).width, (result1.size()).height, (result1.size()).width, (result1.size()).height));
    result1.copyTo(left_top);
    result2.copyTo(right_top);
    result3.copyTo(left_bottom);
    result4.copyTo(right_bottom);

    line( im3,Point((result1.size()).width, 0),Point((result1.size()).width, 2*(result1.size()).height),Scalar( 0, 0, 0 ),3,8 );
    line( im3,Point(0, (result1.size()).height),Point(2*(result1.size()).width, (result1.size()).height),Scalar( 0, 0, 0 ),3,8 );
    putText( im3, "Left(t) and Left(t-1)", Point(0+3,0+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);
    putText( im3, "Left(t-1) and Right(t-1)", Point((result1.size()).width+3, 0+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);
    putText( im3, "Right(t-1) and Right(t)", Point(0+3, (result1.size()).height+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);
    putText( im3, "Right(t) and Left(t)", Point((result1.size()).width+3, (result1.size()).height+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);

    imshow("Feature Correspondence",im3);


//    cout << points1[0].x<<"  "<<points1[0].x<<endl;
    vector<Point> matchindex23;

    vector<pointmatch> mp_new;
    vector<pointmatch> mp_old;


/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Finding all matched points//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

// Left(t-1) and Right(t-1)

    for (int i=0;i<points2.size();i++){
		for(int j=0;j<points3.size();j++){
			if (int(points2[i].x)==int(points3[j].x) && int(points2[i].y)==int(points3[j].y)){
				pointmatch a1;
				a1.point1 = points1[i];
				a1.point2 = points2[i];
				mp_new.push_back(a1);
			}
		}
    }


	mp_old = mp_new;
	mp_new.clear();
	vector<pointmatch>().swap(mp_new);
	
	
    for (int i=0;i<points4.size();i++){
		for (int k=0;k<mp_old.size();k++){
			if (int(mp_old[k].point2.x)==int(points3[i].x) && int(mp_old[k].point2.y)==int(points3[i].y)){
				for(int j=0;j<points5.size();j++){
					if (int(points4[i].x)==int(points5[j].x) && int(points4[i].y)==int(points5[j].y)){
						//cout <<"Mp_old point1 =   "<<mp_old[k].point2<<endl;
						pointmatch a1;
						a1.point1 = mp_old[k].point1;
						a1.point2 = mp_old[k].point2;
						a1.point3 = points5[j];
						mp_new.push_back(a1);	
					}
				}
			}
		}
	}
 				
	mp_old.clear();
	mp_old = mp_new;
	mp_new.clear();
	
	for (int i=0;i<points6.size();i++){
		for (int k=0;k<mp_old.size();k++){
			if (int(mp_old[k].point3.x)==int(points5[i].x) && int(mp_old[k].point3.y)==int(points5[i].y)){
				for(int j=0;j<points7.size();j++){
					if (int(points6[i].x)==int(points7[j].x) && int(points6[i].y)==int(points7[j].y)){
						//cout <<"Mp_old point1 =   "<<mp_old[k].point2<<endl;
						pointmatch a1;
						a1.point1 = mp_old[k].point1;
						a1.point2 = mp_old[k].point2;
						a1.point3 = mp_old[k].point3;
						a1.point4 = points7[j];
						mp_new.push_back(a1);	
					}
				}
			}
		}
	}
 				
	mp_old.clear();
	mp_old = mp_new;
	mp_new.clear();
	

	mp_new.clear();

	for (int i=0;i<points8.size();i++){
		for (int k=0;k<mp_old.size();k++){
			if (int(mp_old[k].point4.x)==int(points7[i].x) && int(mp_old[k].point4.y)==int(points7[i].y)){
				for(int j=0;j<points1.size();j++){
					if (int(points8[i].x)==int(points1[j].x) && int(points8[i].y)==int(points1[j].y)){
						//cout <<"Mp_old point1 =   "<<mp_old[k].point2<<endl;
						pointmatch a1;
						a1.point1 = points1[j];
						a1.point2 = mp_old[k].point2;
						a1.point3 = mp_old[k].point3;
						a1.point4 = mp_old[k].point4;
						mp_new.push_back(a1);	
					}
				}
			}
		}
	}

	mp_old.clear();

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////	
	
	
	cout<< "No. of Matched Points ="<<mp_new.size()<<endl;

	


    Mat check1=left_new.clone();
    Mat check2=left_old.clone();
    Mat check3=right_old.clone();
    Mat check4=right_new.clone();

	for (int i=0;i<mp_new.size();i++){
		circle(check1,mp_new[i].point1,3,Scalar(255,0,0),-1,8,0);
		circle(check2,mp_new[i].point2,3,Scalar(255,0,0),-1,8,0);
		circle(check3,mp_new[i].point3,3,Scalar(255,0,0),-1,8,0);
		circle(check4,mp_new[i].point4,3,Scalar(255,0,0),-1,8,0);
	 }
/*
    for (int i=0;i<matchindex81.size();i++){
	circle(check1,points1[matchindex81[i].y],3,Scalar(255,0,0),-1,8,0);
	
    }

    vector<Point> newfound23;
    for (int i=0;i<matchindex81.size();i++){
    	for (int j=0;j<matchindex23.size();j++){
		if (matchindex81[i].y==matchindex23[j].x){
			circle(check2,points2[matchindex23[j].x],3,Scalar(255,0,0),-1,8,0);
			newfound23.push_back(matchindex23[j]);
		}
    	}
    }

    cout << "81 size = "<< matchindex81.size()<<endl;
    cout << "23 size = "<< newfound23.size()<<endl;
    cout << "*********************"<<endl;
*/

    resize(check1, check1, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
    resize(check2, check2, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
    resize(check3, check3, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );
    resize(check4, check4, Size(0,0), 0.5, 0.5, CV_INTER_LINEAR );

    


	
    Mat new_view(480,640, CV_8UC1);
    Mat lefttop2(new_view, Rect(0, 0, 320, 240));
    Mat leftbottom2(new_view, Rect(0, 240, 320,240));
    Mat righttop2(new_view, Rect(320, 0, 320,240));
    Mat rightbottom2(new_view, Rect(320, 240, 320,240));

    check1.copyTo(lefttop2);
    check2.copyTo(leftbottom2);
    check3.copyTo(righttop2);
    check4.copyTo(rightbottom2);




    line( new_view,Point((check1.size()).width, 0),Point((check1.size()).width, 2*(check1.size()).height),Scalar( 0, 0, 0 ),3,8 );
    line( new_view,Point(0, (check1.size()).height),Point(2*(check1.size()).width, (check1.size()).height),Scalar( 0, 0, 0 ),3,8 );
    putText( new_view, "Left(t) ", Point(0+3,0+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);
    putText( new_view, "Right(t)", Point((check1.size()).width+3, 0+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);
    putText( new_view, "Left(t-1)", Point(0+3, (check1.size()).height+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);
    putText( new_view, "Right(t-1)", Point((check1.size()).width+3, (check1.size()).height+20), 1,1,Scalar( 0, 0, 0 ), 1, 7,false);

	
	
	cout<< "*************************"<<endl;
	Matcher::p_match matched;
	vector<Matcher::p_match> p_matched;
	if (mp_new.size()!=0){
		for (int i = 0;i<int(mp_new.size());i++){
			matched.u1p = mp_new[i].point2.x;
			matched.v1p = mp_new[i].point2.x;
			matched.u2p = mp_new[i].point3.x;
			matched.v2p = mp_new[i].point3.x;
			matched.u1c = mp_new[i].point1.x;
			matched.v1c = mp_new[i].point1.x;
			matched.u2c = mp_new[i].point4.x;
			matched.v2c = mp_new[i].point4.x;
			p_matched.push_back(matched);
		}
	}
	

	/*********************************************************************************************************/

	VisualOdometryStereo viso(params);
	bool success = viso.process(p_matched);
    nav_msgs::Odometry odometry_msg;
    nav_msgs::Odometry odometry_msg1;
    
    Matrix camera_motion;
    if (success){
		Matrix motion = Matrix::inv(viso.getMotion());
		if (change_reference_frame){
			camera_motion = Matrix::inv(reference_motion) * motion;
			}
		else{
			camera_motion = motion;
		}
		reference_motion = motion;
		tf::Matrix3x3 rot_mat(
        camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
        camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
        camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);
		
		
		ros::Time time_stamp = l_info_msg->header.stamp;
		if (iter123==1){ 
			integrated_pose.setIdentity();
			iter123=123;
			}
		
	    integrated_pose *= delta_transform;
	    
		tf::StampedTransform base_to_sensor;
		std::string error_msg;
		
		ros::NodeHandle local_nh("~");
		string sensor_frame_id;
		tf::TransformListener tf_listener;
		string base_link_frame_id;
		local_nh.param("sensor_frame_id", sensor_frame_id, std::string("/camera"));
		local_nh.param("base_link_frame_id", base_link_frame_id, std::string("/base_link"));
		
		if (tf_listener.canTransform(base_link_frame_id, sensor_frame_id, time_stamp, &error_msg))
		{
			tf_listener.lookupTransform(
			  base_link_frame_id,
			  sensor_frame_id,
			  time_stamp, base_to_sensor);
		}
		else
			{
			  ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
									  "will assume it as identity!", 
									  base_link_frame_id.c_str(),
									  sensor_frame_id.c_str());
			  ROS_DEBUG("Transform error: %s", error_msg.c_str());
			  base_to_sensor.setIdentity();
			  
		}
		
		tf::Transform base_transform = base_to_sensor * integrated_pose * base_to_sensor.inverse();
		base_to_sensor.setIdentity();
		tf::Transform base_transform1 = base_to_sensor * integrated_pose * base_to_sensor.inverse();
		
		odometry_msg.header.stamp = time_stamp;
		odometry_msg1.header.stamp = time_stamp;
		tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);
		tf::poseTFToMsg(base_transform1, odometry_msg1.pose.pose);
		cout << odometry_msg.pose.pose.position.x;
		ofstream outfile1;
		const char * c = filename1.c_str();
		outfile1.open(c, std::ios_base::app);
		outfile1 <<odometry_msg.pose.pose.position.x<< "\t"<<odometry_msg.pose.pose.position.y<<"\t"<<odometry_msg.pose.pose.position.z<<"\n";
		ofstream outfile2;
		const char * c2 = filename2.c_str();
		outfile2.open(c2, std::ios_base::app);
		outfile2 <<odometry_msg1.pose.pose.position.x<< "\t"<<odometry_msg1.pose.pose.position.y<<"\t"<<odometry_msg1.pose.pose.position.z<<"\n";
		//integrateAndPublish(delta_transform, l_image_msg->header.stamp);
	}
	 
	 
	
	if(success)
      {
       switch ( ref_frame_change_method)
		{
          case 1:
          {
            double total_flow = 0.0;
			for (size_t i = 0; i < p_matched.size(); ++i)
			{
			  double x_diff = p_matched[i].u1c - p_matched[i].u1p;
			  double y_diff = p_matched[i].v1c - p_matched[i].v1p;
			  total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
			}
			double feature_flow = total_flow / p_matched.size();
            change_reference_frame = (feature_flow < ref_frame_motion_threshold);
            break;
          }
          case 2:
          {
            change_reference_frame = (viso.getNumberOfInliers() > ref_frame_inlier_threshold);
            break;
          }            
          default:
            change_reference_frame = false;
        }
        
      }
	
	else
		change_reference_frame = false;
	
	
	imshow("Features Found Again",new_view);

    waitKey(10); //press any key to quit
    
    }
    iteration++;
    left_old=left_new.clone();
    right_old=right_new.clone();
 //   waitKey(2);
	
    
}	



int main(int argc, char** argv)
{	
	ros::init(argc, argv, "feature_extract");
	ros::NodeHandle nh;
	time_t now = time(0);
	dt = ctime(&now);
	filename1.append("/home/mayank/paths/");
	filename2.append("/home/mayank/paths/");
	filename1.append("withtransform_");
	filename2.append("withouttransform_");
	filename1.append(dt);
	filename2.append(dt);
	filename1.append(".txt");
	filename2.append(".txt");
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
	
// Kitti Database
	/*message_filters::Subscriber<Image> imagel_sub(nh, "/kitti_stereo/left/image_rect", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "/kitti_stereo/right/image_rect", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"/kitti_stereo/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"/kitti_stereo/right/camera_info", 1);*/

// Camera_own
	/*message_filters::Subscriber<Image> imagel_sub(nh, "/stereo/left/image_raw", 1);
	message_filters::Subscriber<Image> imager_sub(nh, "/stereo/right/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfol_sub(nh,"/stereo/left/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>  caminfor_sub(nh,"/stereo/right/camera_info", 1);*/
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imagel_sub,imager_sub,caminfol_sub,caminfor_sub);
	sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3, _4));
//	image_transport::Subscriber imagemaskSub = it->subscribe   ("/riverdetector/visualization" , 1, maskimageCb);
	while (ros::ok())
	{
	

//	ROS_INFO("I published something!");

	ros::spinOnce();
}
  return 0;
}
