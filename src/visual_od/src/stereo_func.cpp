#include "stereo_func.h"

using namespace std;
using namespace cv;

  //~ Matx                         Tr_delta;   // transformation (previous -> current frame)  
 
  vector<pointmatch>  p_matched;  // feature point matches
  
 
  vector<double> X;
	vector<double> Y;
	vector<double> Z;
	vector<double> J;
	vector<double> p_predict;
	vector<double> p_observe;
	vector<double> p_residual;
  parameters param;
  vector<int> inliers;    // inlier set
  Matx44d Tr_delta;
  

int results;



/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Get Camera Parameters//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

parameters getCameraparams(StereoCameraModel model){
	param.cu=model.left().cx();
	param.cv=model.left().cy();
	param.f=model.left().fx();
	param.base = model.baseline();
	//~ cout << "CU =" << param.cu<<endl;
	return param;
} 

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Save Observations//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

void computeObservations(vector<pointmatch> &p_matched,vector<int> &active){
		// set all observations
  p_observe.clear();		
  for (int i=0; i<(int)active.size(); i++) {
    p_observe.push_back(p_matched[active[i]].point1.x); // u1
    p_observe.push_back(p_matched[active[i]].point1.y); // v1
    p_observe.push_back(p_matched[active[i]].point4.x); // u2
    p_observe.push_back(p_matched[active[i]].point4.y); // v2
  }
}

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Calculate Prediction and Jacobian //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

void computeResidualsAndJacobian(vector<double> &tr,vector<int> &active){
	
	p_predict.clear();
	p_residual.clear();
	J.clear();
// extract motion parameters
  double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
  double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

  // compute rotation matrix and derivatives
  double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
  double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*sz; double rdrx12 = -cx*cy;
  double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
  double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
  double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
  double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
  double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
  double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
  double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;
  // for all observations do
  for (int i=0; i<(int)active.size(); i++) {

    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;
    
    // weighting
    double weight = 1.0;
    if (true)
      weight = 1.0/(fabs(p_observe[4*i+0]-param.cu)/fabs(param.cu) + 0.05);
    
    // compute 3d point in current right coordinate system
    X2c = X1c-param.base;

    // for all paramters do
    for (int j=0; j<6; j++) {

      // derivatives of 3d pt. in curr. left coordinates wrt. param j
      switch (j) {
        case 0: X1cd = 0;
                Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                break;
        case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                break;
        case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                Y1cd = rdrz10*X1p+rdrz11*Y1p;
                Z1cd = rdrz20*X1p+rdrz21*Y1p;
                break;
        case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }
	  
      // set jacobian entries (project via K)
      J.push_back(weight*param.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c)); // left u'
      J.push_back(weight*param.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c)); // left v'
      J.push_back(weight*param.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c)); // right u'
      J.push_back(weight*param.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c)); // right v'
    }
	
	
    // set prediction (project via K)
    p_predict.push_back(param.f*X1c/Z1c+param.cu); // left u
    p_predict.push_back(param.f*Y1c/Z1c+param.cv); // left v
    p_predict.push_back(param.f*X2c/Z1c+param.cu); // right u
    p_predict.push_back(param.f*Y1c/Z1c+param.cv); // right v
    
    // set residuals
    p_residual.push_back(weight*(p_observe[4*i+0]-p_predict[4*i+0]));
    p_residual.push_back(weight*(p_observe[4*i+1]-p_predict[4*i+1]));
    p_residual.push_back(weight*(p_observe[4*i+2]-p_predict[4*i+2]));
    p_residual.push_back(weight*(p_observe[4*i+3]-p_predict[4*i+3]));
    
  }
  
}

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Update the global parameters//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

int updateParameters(vector<pointmatch> &p_matched,vector<int> &active,vector<double> &tr,double step_size,double eps){
	if (active.size()<3)
    return 3;

// extract observations and compute predictions
	computeObservations(p_matched,active);
	computeResidualsAndJacobian(tr,active);
	Matx66d A;
	Matx61d B;
	for (int m=0; m<6; m++) {
		for (int n=0; n<6; n++) {
			double a = 0;
			for (int i=0; i<4*(int)active.size(); i++) {
					a += J[i*6+m]*J[i*6+n];
			}
			A(m,n) = a;
		}
		double b = 0;
		for (int i=0; i<4*(int)active.size(); i++) {
			b += J[i*6+m]*(p_residual[i]);
		}
		B(m,0) = b;
	}
	
	Matx61d x;
  // perform elimination
	if (solve(A, B, x,DECOMP_CHOLESKY )) {
		solve(A, B, x,DECOMP_CHOLESKY );
	    B=x;
		bool converged = true;
		for (int m=0; m<6; m++) {
			tr[m] += step_size*B(m,0);
			if (fabs(B(m,0))>eps)
				converged = false;
		}
		
		if (converged)
			return 1;
		else
			return 2;
	}
	else {
		return 3;
	}
}



/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Get randomly selected sample//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

vector<int> selectsample(int N,int p){
	 // init sample and totalset
	vector<int> sample;
	vector<int> totalset;
  
// create vector containing all indices
	for (int i=0; i<N; i++)
		totalset.push_back(i);

	// add num indices to current sample
	sample.clear();
	for (int i=0; i<p; i++) {
		int j = rand()%totalset.size();
		sample.push_back(totalset[j]);
		totalset.erase(totalset.begin()+j);
	}
  
// return sample
	return sample;
	
}


/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									//Get Inliers//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

vector<int> getInlier(vector<pointmatch> &p_matched,vector<double> &tr){
	vector<int> active;
	for (int i=0; i<(int)p_matched.size(); i++){
		active.push_back(i);
	}
	
// extract observations and compute predictions
	computeObservations(p_matched,active);
	computeResidualsAndJacobian(tr,active);
cout<<"Inlier size = "<<p_residual[1]<<endl;
// compute inliers
	
	for (int i=0; i<(int)p_matched.size(); i++){
		double iskaval = pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2)+pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2);
		
		if (iskaval < 1.5*1.5){
			inliers.push_back(i);
		}
	}
	
  return inliers;
}

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									//Estimate Motion//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

vector<double> motionestimation(vector<pointmatch> mp){
	bool success = true;
	double width=0,height=0;
	p_matched = mp;
//Minimum distance for RANSAC Sample
	for (vector<pointmatch>::iterator it=mp.begin(); it!=mp.end(); it++) {
	    if (it->point1.x>width)  width  = it->point1.x;
	    if (it->point1.y>height) height = it->point1.y;
	  }
	double min_dist = min(width,height)/3.0;

//Size of mp
	int size = mp.size();
	
	
	if (size<6){
		return vector<double>();
	}

// allocate dynamic memory
	X.clear();
	Y.clear();
	Z.clear();
		
	for (int i=0; i<size; i++) {
		double d = max(mp[i].point2.x - mp[i].point3.x,1.0f);
		X.push_back((mp[i].point2.x-param.cu)*param.base/d);
		Y.push_back((mp[i].point2.y-param.cv)*param.base/d);
		Z.push_back( param.f*param.base/d);
	}
	
	vector<double> tr_delta;
	vector<double> tr_delta_curr;

	tr_delta_curr.resize(6);
	// clear parameter vector
	inliers.clear();
	
	
	//Iteration for RANSAC
	for (int k=0;k<1000;k++) {
		// selecting random samples
		vector<int> active = selectsample(size,3);
		tr_delta_curr.assign(6,0);
		
		results = 1;
		int iter=0;
		while (results==1) {
			results = updateParameters(mp,active,tr_delta_curr,1,1e-6);
			
			if (iter++ > 20 || results==2){
				break;
			}
		}
		
		/*
		cout << "Value = "<<tr_delta[1]<<endl;
		cout << "Size = "<<p_predict.size()<<endl;
		*/
		if (results!=3) {
			
			vector<int> inliers_curr = getInlier(mp,tr_delta_curr);
			if (inliers_curr.size()>inliers.size()) {
				inliers = inliers_curr;
				tr_delta = tr_delta_curr;
			}
		}
	}
	
	
	
  /*
  // final optimization (refinement)
  if (inliers.size()>=6) {
    int iter=0;
    results = UPDATED;
    while (results==UPDATED) {     
      results = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
      if (iter++ > 100 || results==CONVERGED)
        break;
    }

    // not converged
    if (results!=CONVERGED)
      success = false;

  // not enough inliers
  } else {
    success = false;
  }
	*/
  // release dynamic memory
 
 
	success = true;
	
  // parameter estimate succeeded?
  if (success) return tr_delta;
  else         return vector<double>();
}


/***************************************************
 *************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Transform vector to matrix//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

Matx44d transformVectorToMatrix(vector<double> tr){
  double rx = tr[0];
  double ry = tr[1];
  double rz = tr[2];
  double tx = tr[3];
  double ty = tr[4];
  double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx);
  double cx = cos(rx);
  double sy = sin(ry);
  double cy = cos(ry);
  double sz = sin(rz);
  double cz = cos(rz);

  // compute transformation
  Matx44d Tr;
  Tr(0,0) = +cy*cz;          Tr(0,1) = -cy*sz;          Tr(0,2) = +sy;    Tr(0,3) = tx;
  Tr(1,0) = +sx*sy*cz+cx*sz; Tr(1,1) = -sx*sy*sz+cx*cz; Tr(1,2) = -sx*cy; Tr(1,3) = ty;
  Tr(2,0) = -cx*sy*cz+sx*sz; Tr(2,1) = +cx*sy*sz+sx*cz; Tr(2,2) = +cx*cy; Tr(2,3) = tz;
  Tr(3,0) = 0;               Tr(3,1) = 0;               Tr(3,2) = 0;      Tr(3,3) = 1;
  return Tr;
}

/***************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//Update the Motion//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************************************/

Matx44d updateMotion(vector<pointmatch> mp, parameters par1) {
  
  // Matched points
  p_matched = mp;
  param = par1;
  // estimate motion
  vector<double> tr_delta = motionestimation(p_matched);
  
  // on failure
  if (tr_delta.size()!=6){
    cout << "failed"<<endl;
    return Matx44d::eye();
	}
  
  // set transformation matrix (previous to current frame)
  Tr_delta = transformVectorToMatrix(tr_delta);
  
  // success
  return Tr_delta;
}
