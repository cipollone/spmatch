
#pragma once

#define DEBUGGING

#include "stereo.hpp"


using std::cout;
using std::endl;


void debug(void) {

	// Just testing here

	// Testing StereoImage constructor
	StereoImage s("../tests/cones/disp2.png");
	cout << s << endl;
	


}
