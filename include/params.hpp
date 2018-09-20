
#pragma once


/*****************************************************
* > struct Params                                    *
* A global container for all constants and settings. *
*****************************************************/
struct Params {

	// Def
	enum class OutOfBounds { REPEAT_PIXEL, BLACK_PIXEL, ZERO_COST, ERROR,
			NAN_COST };
	
	// Math constants
	double ALFA;
	double TAU_COL;
	double TAU_GRAD;
	double GAMMA;

	// Range parameters
	unsigned WINDOW_SIZE;          // Must be an odd number
	int MIN_D;
	int MAX_D;
	unsigned ITERATIONS;
	double MAX_SLOPE;              // Max slope of the window

	// Flag parameters
	bool NORMALIZE_GRADIENTS;      // With this false, TAU_GRAD must also change
	OutOfBounds OUT_OF_BOUNDS;
	bool RESIZE_WINDOWS;
	bool PLANES_SATURATION;
	bool USE_PSEUDORAND;
	bool CONST_DISPARITIES;
	int LOG;                       // {0,...,3}. 0 means off

};

extern Params params;		// defined and set in spmatch.cpp
