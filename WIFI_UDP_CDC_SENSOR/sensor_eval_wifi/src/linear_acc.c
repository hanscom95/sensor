// Copyright (c) 2015, Standing Egg, Inc.
// All rights reserved.
// 
// Programmed by GUSTAVO.


#include "linear_acc.h"
#include <math.h>

// value used in the first cyle
float fGrav[3] = {0, 0, 0};		// Gravity

// input 3 elements array
void linearAcc(float *fAccArray, float* fLAccArray) {

	//// Recursive Filter (1st order low-pass filter), first loop starts with fGrav to 0 (no reading before)
	
	// Finds Gravity
	fGrav[X] = ALPHA * fGrav[X] + (1 - ALPHA) * fAccArray[X];
	fGrav[Y] = ALPHA * fGrav[Y] + (1 - ALPHA) * fAccArray[Y];
	fGrav[Z] = ALPHA * fGrav[Z] + (1 - ALPHA) * fAccArray[Z];
	
	// Substracts gravity to find Linear Acceleration
	fLAccArray[X] = fAccArray[X] - fGrav[X];
	fLAccArray[Y] = fAccArray[Y] - fGrav[Y];
	fLAccArray[Z] = fAccArray[Z] - fGrav[Z];
	
	
	return fLAccArray;
}

// (Experimental, do not use)
//float svm(float *fAccArray, float one_gravity) {
//	
//	return sqrt(fAccArray[X]*fAccArray[X] +  fAccArray[Y]* fAccArray[Y] +  fAccArray[Z] *  fAccArray[Z]) - one_gravity;
//	
//}