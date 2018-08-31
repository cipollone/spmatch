// This file is only used for testing and debugging

#include <iostream>
#include <string>

#include <Eigen/Core>

#include "geometry.hpp"


using std::cout;
using std::endl;
using Eigen::Vector3d;


int main(int argc, char** argv) {

	// > Just testing here!
	
	// Testing random plane function
	for (int i = 0; i < 1000; ++i) {
		PlaneFunction p1;
		p1.setRandomFunction(1,2,-3,-2);
		auto planeParams = p1.getParams();
		auto funParams = p1.getFunParams();
		cout << planeParams.first.transpose() << " " << planeParams.second << " ";
		cout << funParams.transpose();
		cout << endl;
	}
}
