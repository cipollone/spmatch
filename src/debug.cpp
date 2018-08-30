// This file is only used for testing and debugging

#include <iostream>
#include <string>

#include "geometry.hpp"
#include "utils.hpp"


using std::cout;
using std::endl;


int main(int argc, char** argv) {

	cout << "Just testing here!" << endl;

	// testing the Grid class with Plane s
	Grid<double> grid2(3,4,3.3);

	cout << grid2;
}
