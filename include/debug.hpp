
#pragma once

#define DEBUGGING

#include "stereo.hpp"


using std::cout;
using std::flush;
using std::endl;


void debug(void) {

	// Just testing here
	
	StereoImage s1("tests/tiny/im2.png", StereoImage::LEFT);
	StereoImage s2("tests/tiny/im6.png", StereoImage::RIGHT);
	s1.bind(&s2);


	/*
	// Old test
	StereoImage s1("tests/cones/im2.png", StereoImage::LEFT);
	StereoImage s2("tests/cones/im6.png", StereoImage::RIGHT);
	s1.bind(&s2);

	cout << s1.pixelTotalCost(100, 141, PlaneFunction()) << endl;
	cout << s1.pixelTotalCost(2, 2, PlaneFunction()) << endl;
	cout << s1.pixelTotalCost(448, 1, PlaneFunction()) << endl;
	cout << s1.pixelTotalCost(449, 373, PlaneFunction()) << endl;
	cout << s1.pixelTotalCost(1, 374, PlaneFunction()) << endl;
	cout << s1.pixelTotalCost(0, 1, PlaneFunction()) << endl;
	*/
}
