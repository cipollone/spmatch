
#pragma once

#define DEBUGGING

#include "stereo.hpp"


using std::cout;
using std::flush;
using std::endl;


void debug(void) {

	// Just testing here

	// Testing StereoImage dissimilarity
	StereoImage s1("tests/cones/im2.png", StereoImage::LEFT);
	StereoImage s2("tests/cones/im6.png", StereoImage::RIGHT);
	s1.bind(&s2);

	// values on the whole line
	std::vector<double> values(228, -1);
	size_t w = 227, h = 133;
	for (size_t d = 0; d < 100; ++d) {
		values[w-d] = s1.pixelDissimilarity(w, h, PlaneFunction(0,0,d));
	}

	for (auto i: values) {
		cout << i << "\n";
	}
	cout << endl;
}
