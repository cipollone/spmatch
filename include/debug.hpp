
#pragma once

#define DEBUGGING

#include "stereo.hpp"


using std::cout;
using std::flush;
using std::endl;


void debug(void) {

	// Just testing here

	// Testing out of bound pixels in the other view
	StereoImage s1("tests/cones/im2.png", StereoImage::LEFT);
	StereoImage s2("tests/cones/im6.png", StereoImage::RIGHT);
	s1.bind(&s2);

	//cout << s1.pixelWindowCost(18, 5, PlaneFunction().setRandomFunction(18,5, 0,20)) << endl;
	
	cout << s1.pixelWindowCost(21, 5, PlaneFunction()) << endl;
	cout << s1.pixelWindowCost(2, 2, PlaneFunction()) << endl;
	cout << s1.pixelWindowCost(448, 1, PlaneFunction()) << endl;
	cout << s1.pixelWindowCost(449, 373, PlaneFunction()) << endl;
	cout << s1.pixelWindowCost(1, 374, PlaneFunction()) << endl;
	cout << s1.pixelWindowCost(0, 1, PlaneFunction()) << endl;

}
