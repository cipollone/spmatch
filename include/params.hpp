
#pragma once

/**************************************************************************
* > namespace Params                                                      *
* A container for constants used in the PatchMatch algorithm.             *
*                                                                         *
* NOTE: these were the paper weights:                                     *
*   const double ALFA = 0.9;                                              *
*   const double TAU_COL = 10;                                            *
*   const double TAU_GRAD = 2;                                            *
*   const double GAMMA = 10;                                              *
*   const unsigned int WINDOW_SIZE = 35;                                  *
* However I can't use them, because I don't know the numeric range of the *
* RGB and gradients.                                                      *
**************************************************************************/
namespace Params {

	// Def
	enum class OutOfBounds { REPEAT_PIXEL, BLACK_PIXEL, ZERO_COST, ERROR,
			NAN_COST };
	
	// Math constants
	const double ALFA = 0.6;
	const double TAU_COL = 50;
	const double TAU_GRAD = 30;
	const double GAMMA = 15;

	// Other parameters
	const unsigned WINDOW_SIZE = 35;       // Must be an odd number
	const bool NORMALIZE_GRADIENTS = true; // With this false, TAU_GRAD must
	                                       //   also change
	const OutOfBounds OUT_OF_BOUNDS = OutOfBounds::NAN_COST;
	const bool resizeWindowWithCosine = true;
	const int MIN_D = 0;
	const int MAX_D = 20;
	//const unsigned ITERATIONS = 3; // TODO
	const unsigned ITERATIONS = 1;
	const int LOG = 3;               // {0,...,3}. 0 means off

} // TODO: let to change these parameters from main()

