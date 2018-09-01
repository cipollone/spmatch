
#include "image.hpp"


// > class Image

/*****************************************
* > Image()                              *
* Constructs and image from a file path. *
*****************************************/
Image::Image(const string& imgPath):
		imgPath(imgPath), img(imgPath.c_str()),
		width(img.width()), height(img.height()), channels(img.spectrum()) {
	
	if (channels != 3) {
		throw std::runtime_error(
				"Wrong image format: " + std::to_string(channels) +
				" channels; only RGB supported");
	}
}


/*************************************************
* > display()                                    *
* Show the image in a new window                 *
*                                                *
* Args:                                          *
*   windowName (string): the title of the window *
*************************************************/
void Image::display(string windowName) {
	CImgDisplay disp(img, windowName.c_str());
	while (!disp.is_closed()) {
		disp.wait();
	}
}


// print
std::ostream& operator<<(std::ostream& out, const Image& img) {
	return out << "Image: " << img.imgPath << ", size: (" << img.width <<
		"," << img.height << "," << img.channels << ")";
}


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
		pixelPlanes(img.width(), img.height(),
			Grid<PlaneFunction>::Order::WIDTH_HEIGHT) {}
		// TODO: width, height order


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
