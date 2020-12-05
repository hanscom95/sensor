// Copyright (c) 2015, Standing Egg, Inc.
// All rights reserved.
// 
// Programmed by GUSTAVO.
//
// 
// 2.- Standard Vector Magnitude (SVM) - 1g  																												/  Result: Acceleration as scalar quantity (Experimental, do not use)

#ifndef  LINEAR_ACC_H
#define LINEAR_ACC_H

// 1st order low-pass filter parameters
#define RC					0.3F  										//  Time constant  / RC
#define DT					(1.0 / 20)							//  Delivery rate (sampling time) in this case 50milisecs
#define ALPHA   (DT / (RC + DT))		//  Determines exactly how much weight to give the previous data vs the raw data

#define X 0
#define Y 1
#define Z 2


// *********************************************************************************
// function prototypes
// ********************************************************************************

// input:	 fAccArray with current acceleration data
// ouput: f:AccArray with calculated linear acceleration data
void linearAcc(float *fAccArray, float* fLAccArray);

// (Experimental, do not use)
// returns the Standad Vector Magnitude (SVM) 
//float svm(float *fAccArray, float one_gravity);

#endif   // #ifndef LINEAR_ACC_H