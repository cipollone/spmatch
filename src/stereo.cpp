
#include "stereo.hpp"


// > class StereoImage

/********************************************************
* > StereoImage()                                       *
* Constructor. Initialize an image for stereo matching. *
* Initlializes the image, the gradient image,           *
* and the array of support windows.                     *
*                                                       *
* Args:                                                 *
*   imgPath (string): the path of the image to load.    *
********************************************************/
StereoImage::StereoImage(const string& imgPath):
		Image(imgPath),
		pixelPlanes(width, height,
			Grid<PlaneFunction>::Order::WIDTH_HEIGHT) {

	// convert to grayscale and store the x gradient
	Image grayscale = toGrayscale();
	imgGradient = grayscale.getCImg().get_gradient("x", 2)(0);
			// NOTE: ^ this is a copy
}


/**************************************************************************
* > getDisparityAt()                                                      *
* Returns the dispariry value at pixel (w,h), as the z-value of the plane *
* at (w,h). NOTE: the plane must be computed, first.                      *
* NOTE: bounds are not checked.                                           *
*                                                                         *
* Args:                                                                   *
*   w (size_t), h (size_t): pixel coordinate                              *
*                                                                         *
* Returns:                                                                *
*   (double): disparity in the continuous domain                          *
**************************************************************************/
double StereoImage::getDisparityAt(size_t w, size_t h) const {

	const PlaneFunction& p = pixelPlanes.get(w, h);
	return p(w,h);
}


/*********************************************************
* > displayGradient()                                    *
* Visualizes the gradient image (renormalized in 0-255). *
*********************************************************/
void StereoImage::displayGradient(void) const {
	auto visualGradient = imgGradient.get_normalize(0,255);
	visualGradient.display(("Gradient img of " + imgPath).c_str());
}


/****************************************************************************
* > bind()                                                                  *
* Associate the current instance to the other one and vice-versa.           *
* Some methods act on the bound instance. Unbind images, first, if bounded. *
*                                                                           *
* Args:                                                                     *
*   o (StereoImage*): pointer to another instance                           *
****************************************************************************/
void StereoImage::bind(StereoImage* o) {

	// checks
	if (o == nullptr || o == this) {
		throw std::logic_error("Bad pointer");
	}
	if (other != nullptr || o->other != nullptr) {
		throw std::logic_error("An instance is bound already");
	}

	// bind
	other = o;
	o->other = this;
}


/***********************
* > unbind()           *
* Undo the bind action *
***********************/
void StereoImage::unbind(void) {

	// checks
	if (other == nullptr) {
		throw std::logic_error("Unbound already");
	}

	// unbind
	other->other = nullptr;
	other = nullptr;
}


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
