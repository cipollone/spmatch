
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
StereoImage::StereoImage(const string& imgPath, Side side):
		Image(imgPath),
		pixelPlanes(width, height,
			Grid<PlaneFunction>::Order::WIDTH_HEIGHT),
		side(side) {

	// convert to grayscale and store the x gradient
	Image grayscale = toGrayscale();
	gradients = grayscale.getCImg().get_gradient("xy", 2);
			// NOTE: ^ this is a copy
	if (Params::NORMALIZE_GRADIENTS) {
		gradients(0).normalize(0,255);
		gradients(1).normalize(0,255);
	}
}


/**************************************************************************
* > disparityAt()                                                         *
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
double StereoImage::disparityAt(size_t w, size_t h) const {

	const PlaneFunction& p = pixelPlanes.get(w, h);
	return p(w,h);
}


/**************************************************************
* > displayGradient()                                         *
* Visualizes the x,y gradient images (renormalized in 0-255). *
**************************************************************/
void StereoImage::displayGradients(void) const {
	auto gradientX = gradients(0).get_normalize(0,255);
	auto gradientY = gradients(1).get_normalize(0,255);
	gradientX.display(("Gradient X of " + imgPath).c_str());
	gradientY.display(("Gradient Y of " + imgPath).c_str());
}


/***************************************************************************
* > pixelDissimilarity()                                                   *
* Computes the disparity function ro(p,q) for p = (w, h) in the current    *
* view (image), and q as the corresponding pixel in the other view.        *
* q has coordinates: (w + disparity(p), h).                                *
* See the reference paper, PatchMatch Stereo, for the function definition. *
* NOTE: requires a bound instance and an RGB image. No bounds check        *
*                                                                          *
* Args:                                                                    *
*   w (size_t), h (size_t): coordinates of the pixel in this image         *
*   disparity (PlaneFunction): the disparity function                      *
*                                                                          *
* Returns:                                                                 *
*   (double): the disparity measure                                        *
***************************************************************************/
double StereoImage::pixelDissimilarity(size_t w, size_t h,
		const PlaneFunction& disparity) const {
	
	if (other == nullptr) {
		throw std::logic_error("Instance not bound");
	}

	// Coordinates of the other pixel
	double d = disparity(w, h);
	int sign;
	switch (side) {
		case LEFT: sign = -1; break;
		case RIGHT: sign = +1; break;
	}
	double qW = w + sign * d;
	double qH = h;

	// This pixel colour and gradient
	double pR = img(w,h,0,0);
	double pG = img(w,h,0,1);
	double pB = img(w,h,0,2);
	double pGrX = gradients(0)(w,h,0,0);
	double pGrY = gradients(1)(w,h,0,0);

	// The other pixel colour and gradient
	double qR = other->at(qW,qH,0);
	double qG = other->at(qW,qH,1);
	double qB = other->at(qW,qH,2);
	double qGrX = other->gradients(0)(qW,qH,0,0);
	double qGrY = other->gradients(1)(qW,qH,0,0);

	// Function
	double gradientDist = std::abs(pGrX - qGrX) +
			std::abs(pGrY - qGrY);  // NOTE: L1
	double colourDist = std::abs(pR - qR) +
			std::abs(pG - qG) +
			std::abs(pB - qB);      // NOTE: L1

	double res = (1 - Params::ALFA) * std::min(colourDist, Params::TAU_COL) +
			Params::ALFA * std::min(gradientDist, Params::TAU_GRAD);
	
	return res;
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
