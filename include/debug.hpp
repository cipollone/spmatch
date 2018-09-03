
#pragma once

#define DEBUGGING

#include "stereo.hpp"


using std::cout;
using std::flush;
using std::endl;


void debug(void) {

	// Just testing here

	// Testing StereoImage binding
	StereoImage s1("../tests/cones/im2.png");
	StereoImage s2("../tests/cones/im6.png");
	StereoImage s3("../tests/cones/im6.png");
	StereoImage s4("../tests/cones/im6.png");

	//s1.bind(0);    cout << "+" << flush;  // this should fail
	//s1.bind(&s1);  cout << "+" << flush;  // this should fail
	s1.bind(&s2);  cout << "+" << flush;  // this should work
	//s1.bind(&s2);  cout << "+" << flush;  // this should fail
	s3.bind(&s4);  cout << "+" << flush;  // this should work
	//s1.bind(&s4);  cout << "+" << flush;  // this should fail
	//s2.bind(&s1);  cout << "+" << flush;  // this should fail
	s2.unbind();   cout << "+" << flush;  // this should work
	s1.bind(&s2);  cout << "+" << flush;  // this should work
	s2.unbind();   cout << "+" << flush;  // this should work
	//s2.unbind();   cout << "+" << flush;  // this should fail
}
