
#pragma once

#define DEBUGGING

#include <chrono>
#include "stereo.hpp"


using std::cout;
using std::flush;
using std::endl;


void debug(void) {

	// Just testing here

	// timing pixelViewPropagation()

	StereoImagePair pair("tests/small/im2.png", "tests/small/im6.png");
	//auto t1 = std::chrono::high_resolution_clock::now();
	pair.leftImg.pixelViewPropagation(10, 30);
	//auto t2 = std::chrono::high_resolution_clock::now();
	//std::chrono::duration<double, std::milli> duration = t2 - t1;
	//cout << duration.count() << endl;

	/*
	StereoImagePair pair("tests/small/im2.png", "tests/small/im6.png");
	Image disparity = pair.computeDisparity();
	disparity.setPath("leftDisparity.png");
	disparity.write();
	*/

}
