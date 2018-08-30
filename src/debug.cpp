// This file is only used for testing and debugging

#include <iostream>
#include <string>

#include "geometry.hpp"


using std::cout;
using std::endl;


int main(int argc, char** argv) {

	cout << "Just testing here!" << endl;

	// Testing PlaneFunction
	PlaneFunction f2(1,-1,3);
	cout << f2 << endl;

	cout << f2(0,0) << endl;
	cout << f2(1,1) << endl;
	cout << f2(1,-1) << endl;

	auto p = f2.toPointAndNorm();
	cout << p.first.transpose() << ", " << p.second.transpose() << endl;

}
