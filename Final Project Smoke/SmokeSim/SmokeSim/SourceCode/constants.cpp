// Written by Peter Kutz.

#include "constants.h"

// TODO: Adjust the various settings below, and add more if you want.  

const int theMillisecondsPerFrame = 10;

#ifdef _DEBUG
//const int theDim[3] = {192, 128, 1};   
const int theDim[3] = {64, 64, 1};
#else
//const int theDim[3] = {164, 200, 50};   
//const int theDim[3] = {192, 128, 1};    
const int theDim[3] = {164, 200, 50};  
//const int theDim[3] = {64, 64, 1};
#endif

const double theCellSize = 0.5;

