
#pragma once

#define DEBUGGING

#include "stereo.hpp"


using std::cout;
using std::flush;
using std::endl;


void debug(void) {

	// Just testing here

	StereoImage s1("tests/cones/im2.png", StereoImage::LEFT);
	StereoImage s2("tests/cones/im2.png", StereoImage::RIGHT);
	s1.bind(&s2);

	// testing gradient interpolation
	for (int d = 0; d < 10; ++d) {
		cout << "pixelDissimilarity (95, 192) -> -" << (d+0) << ": ";
		cout << s1.pixelDissimilarity(95, 192, PlaneFunction(0,0,d)) << endl;
		cout << "pixelDissimilarity (95, 192) -> -" << (d+0.2) << ": ";
		cout << s1.pixelDissimilarity(95, 192, PlaneFunction(0,0,d+0.2)) << endl;
		cout << "pixelDissimilarity (95, 192) -> -" << (d+0.4) << ": ";
		cout << s1.pixelDissimilarity(95, 192, PlaneFunction(0,0,d+0.4)) << endl;
		cout << "pixelDissimilarity (95, 192) -> -" << (d+0.6) << ": ";
		cout << s1.pixelDissimilarity(95, 192, PlaneFunction(0,0,d+0.6)) << endl;
		cout << "pixelDissimilarity (95, 192) -> -" << (d+0.8) << ": ";
		cout << s1.pixelDissimilarity(95, 192, PlaneFunction(0,0,d+0.8)) << endl;
		cout << "pixelDissimilarity (95, 192) -> -" << (d+1) << ": ";
		cout << s1.pixelDissimilarity(95, 192, PlaneFunction(0,0,d+1)) << endl;
	}
}
