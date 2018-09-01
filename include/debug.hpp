
#pragma once

#define DEBUGGING

#include "utils.hpp"
#include "geometry.hpp"
#include "image.hpp"

using std::cout;
using std::endl;


void debug(void) {

	// Just testing here

	// Testing to grayscale
	Image img("../tests/cones/im2.png");
	Image grayImg = img.toGrayscale();

	img.display();
	grayImg.display();

	grayImg.setPath("../grayscale_test.png");
	grayImg.write();
}
