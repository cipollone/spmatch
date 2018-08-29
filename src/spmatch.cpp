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

#include "image.hpp"


namespace optionsLib = boost::program_options;
using std::string;


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

	// Read the two stereo images
	StereoImagePair stereo(leftImgPath, rightImgPath);

	// Run the algorithm
	Image disparity = stereo.computeDisparity();
	
	// Write the result
	// TODO
	
	// TODO: remove this
	disparity.display("Disparity");

}


// main
int main(int argc, char *argv[]) {

	// parsing the command line
	optionsLib::options_description optDescription("SPMatch. "
			"Stereo matching with slanted support windows");

	optDescription.add_options()
			("help,h", "Help")
			("stereo-images", optionsLib::value<std::vector<string>>()->required(),
			"The left and right input images")
			("output,o", optionsLib::value<string>(), "Output file")
	;
	optionsLib::positional_options_description posOptDescription;
	posOptDescription.add("stereo-images", 2);

	optionsLib::variables_map vMap;
	optionsLib::store(optionsLib::command_line_parser(argc, argv)
			.options(optDescription).positional(posOptDescription).run(),
			vMap);
	optionsLib::notify(vMap);

	// help param
	if (vMap.count("help")) {
		std::cout << optDescription << std::endl;
		return 1;
	}

	// images param
	std::vector<string> imagesPath = vMap["stereo-images"]
			.as<std::vector<string>>();
	if (imagesPath.size() < 2) {		// enough?
		throw std::runtime_error("Need two images");
	}
	for (string s: imagesPath) {		// existing?
		std::ifstream f(s);
		if (!f.good()) {
			throw std::runtime_error("File not found: " + s);
		}
	}

	// output param
	string outputPath;

	size_t dotPos = imagesPath[0].find_last_of('.');
	string leftImageName = imagesPath[0].substr(0,dotPos);
	string leftImageExt = (dotPos < imagesPath[0].length()) ?
			imagesPath[0].substr(dotPos) : "";
	outputPath = leftImageName + "_disparity" + leftImageExt;

	if (vMap.count("output")) {
		outputPath = vMap["output"].as<string>();
	}

	// run
	writeDisparityMap(imagesPath[0],  imagesPath[1], outputPath);

	return 1;
}

