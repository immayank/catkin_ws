#ifndef __STEREO_FUNC_H__
#define __STEREO_FUNC_H__

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

using namespace std;
using namespace cv;
using namespace image_geometry;

// Define New Structure
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

struct parameters{
	public:
		double cu;
		double cv;
		double f;
		double base;
};
// Define Global variables


enum result{UPDATED,FAILED,CONVERGED};


parameters getCameraparams(StereoCameraModel model);

vector<int> selectsample(int N,int p);

void computeObservations(vector<pointmatch> &p_matched,vector<int> &active);

void computeResidualsAndJacobian(vector<double> &tr,vector<int> &active);

int updateParameters(vector<pointmatch> &p_matched,vector<int> &active,vector<double> &tr,double step_size,double eps);

vector<int> getInlier(vector<pointmatch> &p_matched,vector<double> &tr);
	
vector<double> motionestimation(vector<pointmatch> mp);

Matx44d transformVectorToMatrix(vector<double> tr);

Matx44d updateMotion(vector<pointmatch> mp, parameters par1);

#endif
