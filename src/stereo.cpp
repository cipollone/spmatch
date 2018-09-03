
#include "stereo.hpp"


// > class StereoImage

/*************************************************************************
* > StereoImage()                                                        *
* Constructor. Initialize an image for stereo matching.  Loads the image *
* and initialize the array of support windows for each pixel.            *
*                                                                        *
* Args:                                                                  *
*   imgPath (string): the path of the image to load.                     *
*************************************************************************/
StereoImage::StereoImage(const string& imgPath):
		Image(imgPath),
		pixelPlanes(width, height,
			Grid<PlaneFunction>::Order::WIDTH_HEIGHT) {}


// > class StereoImagePair

/**********************************
* > displayBoth()                 *
* Show both images in two windows *
**********************************/
void StereoImagePair::displayBoth(void) {
	leftImg.display("Left image");
	rightImg.display("Right image");
}


/********************************************************************
* > computeDisparity()                                              *
* Computes the disparity map of the two images using the PatchMatch *
* Stereo algorithm.                                                 *
*                                                                   *
* Returns:                                                          *
*   (Image): the disparity map                                      *
********************************************************************/
Image StereoImagePair::computeDisparity(void) {
	// TODO: remove the placeholder
	return Image("../tests/cones/all.png");
}


// print
std::ostream& operator<<(std::ostream& out, const StereoImagePair& pair) {
	return out << "StereoImagePair:\n  " << pair.leftImg << "\n  " <<
		pair.rightImg << std::endl;
}
