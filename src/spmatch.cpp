/*******************************************************************************
* This project is the final part of the Seminars course. This is an            *
* implementation of the PatchMatch Stereo algorithm for 3D local stereo        *
* reconstruction. The algorithm is described in the paper:                     *
* Michael Bleyer, Christoph Rhemann and Carsten Rother. PatchMatch Stereo -    *
* Stereo Matching with Slanted Support Windows. In Jesse Hoey, Stephen McKenna *
* and Emanuele Trucco, Proceedings of the British Machine Vision Conference,   *
* pages 14.1-14.11. BMVA Press, September 2011.                                *
* http://dx.doi.org/10.5244/C.25.14                                            *
*******************************************************************************/

#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

#include "params.hpp"


namespace po = boost::program_options;
using std::string;
using std::cout;
using std::endl;


// Global parameters
Params params;


// Forward eclarations
void setDefaults(void);
void writeDisparityMap(const string& leftImgPath, const string& rightImgPath,
		const string& disparityPath);
void debug(void);


// main
int main(int argc, char *argv[]) {

	// Default settings
	setDefaults();

	// General options
	string outputPath;
	std::vector<string> inputImages;

	// parsing the command line
	po::options_description generalOpts("General options");
	po::options_description paramsOpts("Parameters");

	generalOpts.add_options()
			("help,h", "Help")
			("output,o", po::value<string>(&outputPath)->default_value(
				"disparity.png"), "Output file")
			("inputs,I", po::value<std::vector<string>>(&inputImages)
				->multitoken()->required(), "Left/right images")
			("log,l", po::value<int>(&params.LOG), "Log level {0,...,3}")
	;
	paramsOpts.add_options()
			("alfa", po::value<double>(&params.ALFA), "ALFA constant")
			("tau_col", po::value<double>(&params.TAU_COL), "TAU_COL constant")
			("tau_grad", po::value<double>(&params.TAU_GRAD), "TAU_GRAD constant")
			("gamma", po::value<double>(&params.GAMMA), "GAMMA constant")
			("window_size,w", po::value<unsigned>(&params.WINDOW_SIZE),
				"Pixel size of the matching window")
			("min_d,m", po::value<int>(&params.MIN_D), "Minimum disparity")
			("max_d,M", po::value<int>(&params.MAX_D), "Maximum disparity")
			("iteration,i", po::value<unsigned>(&params.ITERATIONS),
				"Number of iterations per view")
			("normalize_gradients", po::value<bool>(
				&params.NORMALIZE_GRADIENTS)->implicit_value(true),
				"Whether the gradient map should be normalized")
			("out_of_bounds", po::value<Params::OutOfBounds>(&params.OUT_OF_BOUNDS),
				"Out of bounds action. One of {repeat, black, zero, error, nan}")
			("resize_window", po::value<bool>(&params.resizeWindowWithCosine)
				->implicit_value(true),
				"Whether slanted windows should be smaller")
			("planes_saturation", po::value<bool>(&params.planesSaturation)
				->implicit_value(true),
				"Force any internal value to be saturated")
	;

	po::positional_options_description positionalOpts;
	positionalOpts.add("inputs", 2);

	po::options_description allOpts(string() + "SPMatch. " + 
			"Stereo matching with slanted support windows (by Cipollone R.)\n" +
			"Usage:\n"+
			"  spmatch <left_image> <right_image>\n");
	allOpts.add(generalOpts).add(paramsOpts);

	po::variables_map vMap;
	po::store(po::command_line_parser(argc, argv).options(
			allOpts).positional(positionalOpts).run(), vMap);

	// Help
	if (vMap.count("help")) {
		std::cout << allOpts << std::endl;
		return 1;
	}

	// Check options
	po::notify(vMap);

	if (inputImages.size() != 2) {		// enough?
		throw std::runtime_error("Need two images");
	}
	for (string s: inputImages) {		// existing?
		std::ifstream f(s);
		if (!f.good()) {
			throw std::runtime_error("File not found: " + s);
		}
	}

	// run TODO attach
	//writeDisparityMap(imagesPath[0],  imagesPath[1], outputPath);

	return 0;
}


/**************************************************************************
* > setDefaults()                                                         *
* Set the defaults values in the global 'params' struct                   *
* NOTE: these were the paper weights:                                     *
*   const double ALFA = 0.9;                                              *
*   const double TAU_COL = 10;                                            *
*   const double TAU_GRAD = 2;                                            *
*   const double GAMMA = 10;                                              *
*   const unsigned int WINDOW_SIZE = 35;                                  *
* However I can't use them, because I don't know the numeric range of the *
*   RGB and gradients.                                                    *
**************************************************************************/
void setDefaults(void) {

	// Math constants
	params.ALFA = 0.4;
	params.TAU_COL = 60;
	params.TAU_GRAD = 30;
	params.GAMMA = 15;

	// Range parameters
	params.WINDOW_SIZE = 15;      // Must be an odd number TODO was 35
	params.MIN_D = 0;
	params.MAX_D = 50;
	params.ITERATIONS = 1;        // TODO the paper uses 3

	// Flag parameters
	params.NORMALIZE_GRADIENTS = true; // With this false, TAU_GRAD must also change
	params.OUT_OF_BOUNDS = Params::OutOfBounds::NAN_COST;
	params.resizeWindowWithCosine = true;
	params.planesSaturation = false;
	params.LOG = 2;               // {0,...,3}. 0 means off

}


/**************************************************************************
* > writeDisparityMap()                                                   *
* Given a pair of stereo images, saves the generated disparity map to the *
* ouput path.                                                             *
*                                                                         *
* Args:                                                                   *
*   leftImgPath (string): left image name/path                            *
*   rightImgPath (string): right image name/path                          *
*   disparityPath (string): output image name/path                        *
**************************************************************************/
void writeDisparityMap(const string& leftImgPath, const string& rightImgPath,
		const string& disparityPath) {

	// TODO
	/*
	// Read the two stereo images
	StereoImagePair stereo(leftImgPath, rightImgPath);

	// Run the algorithm
	Image disparity = stereo.computeDisparity();
	
	// Write the result
	// todo
	
	// todo: remove this
	disparity.display("Disparity");
	*/

}


void debug(void) {

	// Just testing here
	//
	// Testing the input files
	
}


/***************************************
* > operator>>                         *
* Utility function to set OutOfBounds. *
***************************************/
std::istream& operator>>(std::istream& in, Params::OutOfBounds& selection) {

	std::string token;
	in >> token;
	if (token == "repeat") {
		selection = Params::OutOfBounds::REPEAT_PIXEL;
	} else if (token == "black") {
		selection = Params::OutOfBounds::BLACK_PIXEL;
	} else if (token == "zero") {
		selection = Params::OutOfBounds::ZERO_COST;
	} else if (token == "error") {
		selection = Params::OutOfBounds::ERROR;
	} else if (token == "nan") {
		selection = Params::OutOfBounds::NAN_COST;
	} else {
		throw std::runtime_error("Invalid OutOfBounds selection");
	}

	return in;
}

