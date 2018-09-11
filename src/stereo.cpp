
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
		image(imgPath),
		gradientX(0,0,1),
		gradientY(0,0,1),
		width(image.size(0)),
		height(image.size(1)),
		disparityPlanes(width, height, Grid<PlaneFunction>::Order::WIDTH_HEIGHT),
		side(side) {

	// convert to grayscale and store the gradients
	Image grayscale = image.toGrayscale();
	CImgList<double> gradients;
	grayscale.getCImg().get_gradient("xy", 2).move_to(gradients);

	if (Params::NORMALIZE_GRADIENTS) {
		gradients(0).normalize(0,255);
		gradients(1).normalize(0,255);
	}

	gradientX = std::move(gradients(0));
	gradientY = std::move(gradients(1));
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

	const PlaneFunction& p = disparityPlanes.get(w, h);
	return p(w,h);
}


/**************************************************************
* > displayGradients()                                        *
* Visualizes the x,y gradient images (renormalized in 0-255). *
**************************************************************/
void StereoImage::displayGradients(void) const {
	gradientX.display("Gradient X");
	gradientY.display("Gradient Y");
}


/***************************************************************************
* > pixelDissimilarity()                                                   *
* Computes the disparity function ro(p,q) for p = (w, h) in the current    *
* view (image), and q as the corresponding pixel in the other view.        *
* q has coordinates: (w + disparity(p), h).                                *
* See the reference paper, PatchMatch Stereo, for the function definition. *
* NOTE: requires a bound instance and an RGB image.                        *
* NOTE: the result for out-of-bounds coordinates is specified by           *
* Params::OUT_OF_BOUNDS.                                                   *
* NOTE: pay attention at implicit conversions int -> size_t.               *
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
	
	// checks
	if (other == nullptr) {
		throw std::logic_error("Instance not bound");
	}
	if (w >= width || h >= height) {
		throw std::runtime_error("Out of bounds (" + std::to_string(w) +
				", " + std::to_string(h) + ")");
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

	// Colour and gradient of this pixel
	double pR = image.get(w,h,0);
	double pG = image.get(w,h,1);
	double pB = image.get(w,h,2);
	double pGrX = gradientX.get(w,h);
	double pGrY = gradientY.get(w,h);

	// Is (qW, qH) out of the image?
	bool qIsOut = false;
	if (qW < 0 || qW >= other->width) {
		qIsOut = true;

		switch (Params::OUT_OF_BOUNDS) {
			case Params::OutOfBounds::ZERO_COST:
				return 0;
			case Params::OutOfBounds::ERROR:
				throw std::logic_error("The pixel (" + std::to_string(qW) + ", " +
						std::to_string(qH) + ") in the other view is out of bounds");
			case Params::OutOfBounds::REPEAT_PIXEL:
				qIsOut = false;		// false means solved; no 'break;' here
			case Params::OutOfBounds::BLACK_PIXEL:
				if (qW < 0) { qW = 0; }
				else if (qW >= other->width) { qW = other->width-1; }
		}
	}

	// Colour and gradient of the other pixel
	double qR = other->image.at(qW,qH,0);
	double qG = other->image.at(qW,qH,1);
	double qB = other->image.at(qW,qH,2);
	double qGrX = other->gradientX.at(qW,qH,0);
	double qGrY = other->gradientY.at(qW,qH,0);

	// Is (qW, qH) out of the image?
	if (qIsOut && Params::OUT_OF_BOUNDS == Params::OutOfBounds::BLACK_PIXEL) {
		qR = 0; qG = 0; qB = 0;
		qGrX = 0; qGrY = 0;
	}

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


/***************************************************************************
* > adaptiveWeight()                                                       *
* Computes the weight function w(p1,p2). See equation (4) in the reference *
* paper, PatchMatch Stereo. It returns a weight (0,1] that is higer for    *
* pixels with similar colour. An idea called adaptive support weight       *
* function. The two pixels are from this image.                            *
*                                                                          *
* Args:                                                                    *
*   w1 (size_t): width coordinate of the first point.                      *
*   h1 (size_t): height coordinate of the first point.                     *
*   w2 (size_t): width coordinate of the second point.                     *
*   h2 (size_t): height coordinate of the second point.                    *
*                                                                          *
* Returns:                                                                 *
*   (double): the weight in (0,1]                                          *
***************************************************************************/
double StereoImage::adaptiveWeight(size_t w1, size_t h1, size_t w2, size_t h2)
		const {

	// checks
	if (w1 >= width || h1 >= height) {
		throw std::runtime_error("Out of bounds (" + std::to_string(w1) +
				", " + std::to_string(h1) + ")");
	}
	if (w2 >= width || h2 >= height) {
		throw std::runtime_error("Out of bounds (" + std::to_string(w2) +
				", " + std::to_string(h2) + ")");
	}

	// Colour of the first point
	double p1R = image.get(w1,h1,0);
	double p1G = image.get(w1,h1,1);
	double p1B = image.get(w1,h1,2);

	// Colour of the second point
	double p2R = image.get(w2,h2,0);
	double p2G = image.get(w2,h2,1);
	double p2B = image.get(w2,h2,2);

	double colourDist = std::abs(p1R - p2R) +
			std::abs(p1G - p2G) +
			std::abs(p1B - p2B);

	return std::exp(-colourDist/Params::GAMMA);
}


/****************************************************************************
* > pixelTotalCost()                                                        *
* Computes the total matching cost for pixel (w,h). The total cost is the   *
* sum of all matching costs for each pixel in a square window around (w,g). *
* Each pixel in the window is matched againts a pixel in the other view,    *
* according to the disparity function of (w,h).                             *
* NOTE: Out of bounds pixels are ignored.                                   *
*                                                                           *
* Args:                                                                     *
*   w (size_t): width coordinate of the pixel                               *
*   h (size_t): height coordinate of the pixel                              *
*                                                                           *
* Returns:                                                                  *
*   (double): window matching cost                                          *
****************************************************************************/
double StereoImage::pixelTotalCost(size_t w, size_t h) const {
	
	// checks
	if (w >= width || h >= height) {
		throw std::runtime_error("Out of bounds (" + std::to_string(w) +
				", " + std::to_string(h) + ")");
	}

	// Setting the extremes of the window
	unsigned halfSide = Params::WINDOW_SIZE / 2; // NOTE: assuming WINDOW_SIZE
	                                             // is an odd number
	size_t minW = (w > halfSide) ? (w - halfSide) : 0;
	size_t maxW = (w + halfSide >= width) ? (width - 1) : (w + halfSide);
	size_t minH = (h > halfSide) ? (h - halfSide) : 0;
	size_t maxH = (h + halfSide >= height) ? (height - 1) : (h + halfSide);

	// The disparity plane
	const PlaneFunction& disparity = disparityPlanes.get(w, h);

	// Scan each pixel in the window
	double totalCost = 0;
	for (size_t iW = minW; iW <= maxW; ++iW) {
		for (size_t iH = minH; iH <= maxH; ++iH) {

			// Accumulate the total with the current pixel
			totalCost += adaptiveWeight(w, h, iW, iH) *
					pixelDissimilarity(iW, iH, disparity);
		}
	}

	// Normalizing the cost
	int nPixels = static_cast<int>(maxW+1-minW) * static_cast<int>(maxH+1-minH);
	double total = totalCost / nPixels;

	return total;
}


/*********************************************************************
* > getDisparityMap()                                                *
* Produces an Image with the disparity map given by disparityPlanes. *
*                                                                    *
* Returns:                                                           *
*   (Image): the disparity for each pixel, as a grayscale image      *
*********************************************************************/
Image StereoImage::getDisparityMap(void) const {

	Image disp(width, height, 1);
	for (size_t w = 0; w < width; ++w) {
		for (size_t h = 0; h < height; ++h) {
			disp(w, h) = disparityAt(w, h);
		}
	}

	return disp;
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


/**********************************************************************
* > setRandomDisparities()                                            *
* Set all 'disparityPlanes' to random linear functions. The range of  *
* disparity values in the central pixel of each plane is given by the *
* parameters [Params::MIN_D, Params::MAX_D]                           *
**********************************************************************/
void StereoImage::setRandomDisparities(void) {

	for (size_t w = 0; w < width; ++w) {
		for (size_t h = 0; h < height; ++h) {
			disparityPlanes(w, h).setRandomFunction(w, h,
					Params::MIN_D, Params::MAX_D);
		}
	}
}


// > class StereoImagePair

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

