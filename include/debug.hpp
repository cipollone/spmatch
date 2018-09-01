
#pragma once

#define DEBUGGING

#include "utils.hpp"
#include "geometry.hpp"
#include "image.hpp"

using std::cout;
using std::endl;


void debug(void) {

	// Just testing here
	
	// Testing the correct initialization of order
	Grid<PlaneFunction> grid1(3, 2, Grid<PlaneFunction>::Order::ROWS_COLS);
	Grid<PlaneFunction> grid2(3, 2, Grid<PlaneFunction>::Order::WIDTH_HEIGHT);

	grid1(2,0) = {3,3,2};
	grid2(2,0) = {3,3,2};
	
	cout << grid1 << endl;
	cout << endl;
	cout << grid2 << endl;

	cout << grid1(4) << endl;
	cout << grid1(4) << endl;

}
