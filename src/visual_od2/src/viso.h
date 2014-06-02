

#ifndef VISO_H
#define VISO_H

#include "matrix.h"
#include "matcher.h"
#include <iostream>

class VisualOdometry {

public:

  // camera parameters (all are mandatory / need to be supplied)
  struct calibration {  
    double f;  // focal length (in pixels)
    double cu; // principal point (u-coordinate)
    double cv; // principal point (v-coordinate)
    calibration () {
      f  = 1;
      cu = 0;
      cv = 0;
    }
  };
  
  
  // general parameters
  struct parameters {
    
    VisualOdometry::calibration calib;            // camera calibration parameters
  };

  // constructor, takes as input a parameter structure:
  // do not instanciate this class directly, instanciate VisualOdometryMono
  // or VisualOdometryStereo instead!
  VisualOdometry (parameters param);
  
  // deconstructor
  ~VisualOdometry ();

  // call this function instead of the specialized ones, if you already have
  // feature matches, and simply want to compute visual odometry from them, without
  // using the internal matching functions.
  bool process (std::vector<Matcher::p_match> p_matched_) {
    p_matched = p_matched_;
    return updateMotion();
  }

  // returns transformation from previous to current coordinates as a 4x4
  // homogeneous transformation matrix Tr_delta, with the following semantics:
  // p_t = Tr_delta * p_ {t-1} takes a point in the camera coordinate system
  // at time t_1 and maps it to the camera coordinate system at time t.
  // note: getMotion() returns the last transformation even when process()
  // has failed. this is useful if you wish to linearly extrapolate occasional
  // frames for which no correspondences have been found
  Matrix getMotion () { return Tr_delta; }

  // returns the number of successfully matched points, after bucketing
  int32_t getNumberOfMatches () { return p_matched.size(); }
  
  // returns the number of inliers: num_inliers <= num_matched
  int32_t getNumberOfInliers () { return inliers.size(); }
  
  // returns the indices of all inliers
  std::vector<int32_t> getInlierIndices () { return inliers; }

  // streams out the current transformation matrix Tr_delta 
  friend std::ostream& operator<< (std::ostream &os,VisualOdometry &viso) {
    Matrix p = viso.getMotion();
    os << p.val[0][0] << " " << p.val[0][1] << " "  << p.val[0][2]  << " "  << p.val[0][3] << " ";
    os << p.val[1][0] << " " << p.val[1][1] << " "  << p.val[1][2]  << " "  << p.val[1][3] << " ";
    os << p.val[2][0] << " " << p.val[2][1] << " "  << p.val[2][2]  << " "  << p.val[2][3];
    return os;
  }
  
protected:

  // calls bucketing and motion estimation
  bool updateMotion ();

  // compute transformation matrix from transformation vector  
  Matrix transformationVectorToMatrix (std::vector<double> tr);

  // compute motion from previous to current coordinate system
  // if motion could not be computed, resulting vector will be of size 0
  virtual std::vector<double> estimateMotion (std::vector<Matcher::p_match> p_matched) = 0;
  
  // get random and unique sample of num numbers from 1:N
  std::vector<int32_t> getRandomSample (int32_t N,int32_t num);

  Matrix                         Tr_delta;   // transformation (previous -> current frame)  
  Matcher                       *matcher;    // feature matcher
  std::vector<int32_t>           inliers;    // inlier set
  double                        *J;          // jacobian
  double                        *p_observe;  // observed 2d points
  double                        *p_predict;  // predicted 2d points
  std::vector<Matcher::p_match>  p_matched;  // feature point matches
  
private:
  
  parameters                    param;     // common parameters
};

#endif // VISO_H

