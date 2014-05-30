#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>

#include "matrix.h"

class Matcher {

public:

  // parameter settings
  struct parameters {
  
    
  };

  // constructor (with default parameters)
  //Matcher(parameters param);

  // deconstructor
 // ~Matcher();

  // structure for storing matches
  struct p_match {
    float   u1p,v1p; // u,v-coordinates in previous left  image
    int32_t i1p;     // feature index (for tracking)
    float   u2p,v2p; // u,v-coordinates in previous right image
    int32_t i2p;     // feature index (for tracking)
    float   u1c,v1c; // u,v-coordinates in current  left  image
    int32_t i1c;     // feature index (for tracking)
    float   u2c,v2c; // u,v-coordinates in current  right image
    int32_t i2c;     // feature index (for tracking)
    p_match(){}
    p_match(float u1p,float v1p,int32_t i1p,float u2p,float v2p,int32_t i2p,
            float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c):
            u1p(u1p),v1p(v1p),i1p(i1p),u2p(u2p),v2p(v2p),i2p(i2p),
            u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c) {}
  };

  // computes features from left/right images and pushes them back to a ringbuffer,
  // which interally stores the features of the current and previous image pair
  // use this function for stereo or quad matching
  // input: I1,I2 .......... pointers to left and right image (row-aligned), range [0..255]
  //        dims[0,1] ...... image width and height (both images must be rectified and of same size)
  //        dims[2] ........ bytes per line (often equals width)
  //        replace ........ if this flag is set, the current image is overwritten with
  //                         the input images, otherwise the current image is first copied
  //                         to the previous image (ring buffer functionality, descriptors need
  //                         to be computed only once)    
  

private:

  // structure for storing interest points
  struct maximum {
    int32_t u;   // u-coordinate
    int32_t v;   // v-coordinate
    int32_t val; // value
    int32_t c;   // class
    int32_t d1,d2,d3,d4,d5,d6,d7,d8; // descriptor
    maximum() {}
    maximum(int32_t u,int32_t v,int32_t val,int32_t c):u(u),v(v),val(val),c(c) {}
  };
  
  // u/v ranges for matching stage 0-3
  struct range {
    float u_min[4];
    float u_max[4];
    float v_min[4];
    float v_max[4];
  };
  
  struct delta {
    float val[8];
    delta () {}
    delta (float v) {
      for (int32_t i=0; i<8; i++)
        val[i] = v;
    }
  };
  

  // compute sparse set of features from image
  // inputs:  I ........ image
  //          dims ..... image dimensions [width,height]
  //          n ........ non-max neighborhood
  //          tau ...... non-max threshold
  // outputs: max ...... vector with maxima [u,v,value,class,descriptor (128 bits)]
  //          I_du ..... gradient in horizontal direction
  //          I_dv ..... gradient in vertical direction
  // WARNING: max,I_du,I_dv has to be freed by yourself!
 
  // parameters
  parameters param;
  int32_t    margin;
  
  int32_t *m1p1,*m2p1,*m1c1,*m2c1;
  int32_t *m1p2,*m2p2,*m1c2,*m2c2;
  int32_t n1p1,n2p1,n1c1,n2c1;
  int32_t n1p2,n2p2,n1c2,n2c2;
  uint8_t *I1p,*I2p,*I1c,*I2c;
  uint8_t *I1p_du,*I2p_du,*I1c_du,*I2c_du;
  uint8_t *I1p_dv,*I2p_dv,*I1c_dv,*I2c_dv;
  uint8_t *I1p_du_full,*I2p_du_full,*I1c_du_full,*I2c_du_full; // only needed for
  uint8_t *I1p_dv_full,*I2p_dv_full,*I1c_dv_full,*I2c_dv_full; // half-res matching
  int32_t dims_p[3],dims_c[3];

  std::vector<Matcher::p_match> p_matched_1;
  std::vector<Matcher::p_match> p_matched_2;
  std::vector<Matcher::range>   ranges;
};

#endif

