
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

Mat old_gray;
int iteration=1;

void imageCallback(const ImageConstPtr& imagel,const ImageConstPtr& imager)
{

	
	cv_bridge::CvImagePtr origl_img;
	origl_img = cv_bridge::toCvCopy(imagel, enc::BGR8);
	
	cv::Mat new_img = origl_img->image;
	Mat new_gray,disp,disp8_sgbm,disp8_sbm;
	cvtColor(new_img, new_gray, CV_BGR2GRAY);
	
	if (iteration>2){
	cout<<iteration;
// image read
    Mat tmp = new_gray;
    Mat in = old_gray;
    
//	Mat left_mask;
//	createTrackbar( "Derx","check", &der_x,10, NULL );
//	createTrackbar( "Dery","check", &der_y,10, NULL );
//	createTrackbar( "Kernel","check", &kernel,3, NULL );
//	Sobel(left_img,left_mask,CV_8U,der_x+1,der_y+1,(2*kernel+1));
// 	imshow("check",left_mask);
    /* threshold      = 0.04;
       edge_threshold = 10.0;
       magnification  = 3.0;    */
 

//	Mat left_blur;
//	Mat left_edge;
//	Mat right_blur;
//	Mat right_edge;
//   blur( left_gray, left_blur, Size(5,5) );
// blur( right_gray, right_blur, Size(5,5) );
////   createTrackbar( "Value","Edge", &value,1000, NULL );
//// createTrackbar( "Ratio","Edge", &ratio,10, NULL );
//   value =14, ratio=1;
//   Canny( left_blur, left_edge, value, (ratio+1)*value, 3 );
//   Canny( right_blur, right_edge, value, (ratio+1)*value, 3 );
//	imshow("Edge",left_edge);
    

    // SIFT feature detector and feature extractor
//    cv::SiftFeatureDetector detector( 0.05, 5.0 );
//    cv::SiftDescriptorExtractor extractor( 3.0 );
 

// In case of SURF, you apply the below two lines
 //   createTrackbar( "Value","SIFT", &value,10000, NULL );
  //  cv::SurfFeatureDetector detector(1500);
   // cv::SurfDescriptorExtractor extractor;


    //FAST feature detector 
    // cv::FastFeatureDetector detector(1500);
    // cv::FastDescriptorExtractor extractor;
     
    ORB orb;
    // Feature detection
    Mat descriptor1,descriptor2;
    vector<KeyPoint> keypoints1;
    vector<KeyPoint> keypoints2;
    orb(tmp,Mat(),keypoints1,descriptor1);
    orb(in,Mat(),keypoints2,descriptor2);
    
    // Feature display
    Mat feat1,feat2;
    drawKeypoints(tmp,keypoints1,feat1,Scalar(255, 255, 255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(in,keypoints2,feat2,Scalar(255, 255, 255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    imwrite( "feat1.bmp", feat1 );
//    imwrite( "feat2.bmp", feat2 );
    int key1 = keypoints1.size();
    int key2 = keypoints2.size();
    printf("Keypoint1=%d \nKeypoint2=%d", key1, key2);
 
    
    // corresponded points
	vector< vector< DMatch >  > matches1;
	vector< vector< DMatch >  > matches2;
 
    // L2 distance based matching. Brute Force Matching
    BFMatcher matcher(NORM_L1); 
 
    // Flann-based matching
   // FlannBasedMatcher matcher;
 
    // display of corresponding points
    matcher.knnMatch( descriptor1, descriptor2, matches1 ,2);
    matcher.knnMatch( descriptor2, descriptor1, matches2 ,2);
     
    // matching result
    Mat result1;
    drawMatches( tmp, keypoints1, in, keypoints2, matches1, result1 );
/***************************************************************************/


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

    Mat result2;
    cout<< symMatches.size()<<endl;
    drawMatches( tmp, keypoints1, in, keypoints2, symMatches, result2 );

///////////
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
  Mat result3;	
  drawMatches( tmp, keypoints1, in, keypoints2, outMatches, result3 );


///////////////////////////////





    //std::vector<uchar> inliers(points1.size(),0);

  //  cv::Mat fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), CV_FM_RANSAC, 3,0.95,inliers,); 

 
    // output file
 //   imwrite( "result.bmp", result );
 
    // display the result
//    namedWindow("SIFT", CV_WINDOW_AUTOSIZE );
    imshow("Before", result1);
    imshow("After", result2);
    imshow("After Ransac", result3);
    waitKey(10); //press any key to quit
    
    }
    iteration++;
    old_gray=new_gray;
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
