
#include "stereo.hpp"

#include "params.hpp"
#include "log.hpp"


using std::to_string;


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

	if (params.NORMALIZE_GRADIENTS) {
		gradients(0).normalize(0,255);
		gradients(1).normalize(0,255);
	}

	gradientX = std::move(gradients(0));
	gradientY = std::move(gradients(1));
}


/**************************************************************************
* > disparityAt()                                                         *
* Returns the dispariry value at pixel (w,h), as the z-value of the plane *
* at (w,h). The z-value is saturated in [MIN_D, MAX_D].                   *
* NOTE: the plane must be computed, first.                                *
* NOTE: bounds are not checked.                                           *
*                                                                         *
* Args:                                                                   *
*   w (size_t), h (size_t): pixel coordinate                              *
*                                                                         *
* Returns:                                                                *
*   (double): disparity in the continuous domain                          *
**************************************************************************/
double StereoImage::disparityAt(size_t w, size_t h) const {

	double d = disparityPlanes.get(w, h)(w, h);
	if (d > params.MAX_D) d = params.MAX_D;
	if (d < params.MIN_D) d = params.MIN_D;

	return d;
}


/**************************************************************
* > displayGradients()                                        *
* Visualizes the x,y gradient images (renormalized in 0-255). *
**************************************************************/
void StereoImage::displayGradients(void) const {
	gradientX.display();
	gradientY.display();
}


/***************************************************************************
* > pixelDissimilarity()                                                   *
* Computes the disparity function ro(p,q) for p = (w, h) in the current    *
* view (image), and q as the corresponding pixel in the other view.        *
* q has coordinates: (w + disparity(p), h).                                *
* See the reference paper, PatchMatch Stereo, for the function definition. *
* NOTE: requires a bound instance and an RGB image.                        *
* NOTE: the result for out-of-bounds coordinates is specified by           *
* params.OUT_OF_BOUNDS.                                                    *
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
		throw std::logic_error("pixelDissimilarity(). Instance not bound");
	}
	if (w >= width || h >= height) {
		throw std::range_error("pixelDissimilarity(). Out of bounds (" +
				to_string(w) + ", " + to_string(h) + ")");
	}

	// Disparity
	double d = disparity(w, h);

	if (params.PLANES_SATURATION) { // Plane saturation on/off
		if (d > params.MAX_D) d = params.MAX_D;
		if (d < params.MIN_D) d = params.MIN_D;
	}

	// Coordinates of the other pixel
	int sign;
	switch (side) {
		case LEFT: sign = -1; break;
		case RIGHT: sign = +1; break;
	}
	double qW = w + sign * d;
	double qH = h;

	// Is (qW, qH) out of the image?
	bool qIsOut = false;
	if (qW < 0 || qW > (other->width-1)) {
		qIsOut = true;

		switch (params.OUT_OF_BOUNDS) {
			case Params::OutOfBounds::ZERO_COST:
				return 0;
			case Params::OutOfBounds::NAN_COST:
				return std::numeric_limits<double>::quiet_NaN();
			case Params::OutOfBounds::ERROR:
				throw std::range_error("pixelDissimilarity(). The pixel (" +
						to_string(qW) + ", " + to_string(qH) +
						") in the other view is out of bounds");
			case Params::OutOfBounds::REPEAT_PIXEL:
				qIsOut = false;		// false means solved; no 'break;' here
			case Params::OutOfBounds::BLACK_PIXEL:
				if (qW < 0) { qW = 0; }
				else if (qW >= other->width) { qW = other->width-1; }
		}
	}

	// Colour and gradient of this pixel
	double pR = image.get(w,h,0);
	double pG = image.get(w,h,1);
	double pB = image.get(w,h,2);
	double pGrX = gradientX.get(w,h);
	double pGrY = gradientY.get(w,h);

	// Colour and gradient of the other pixel
	double qR = other->image.at(qW,qH,0);
	double qG = other->image.at(qW,qH,1);
	double qB = other->image.at(qW,qH,2);
	double qGrX = other->gradientX.at(qW,qH,0);
	double qGrY = other->gradientY.at(qW,qH,0);

	// Is (qW, qH) out of the image?
	if (qIsOut && params.OUT_OF_BOUNDS == Params::OutOfBounds::BLACK_PIXEL) {
		qR = 0; qG = 0; qB = 0;
		qGrX = 0; qGrY = 0;
	}

	// Function
	double gradientDist = std::abs(pGrX - qGrX) +
			std::abs(pGrY - qGrY);  // NOTE: L1
	double colourDist = std::abs(pR - qR) +
			std::abs(pG - qG) +
			std::abs(pB - qB);      // NOTE: L1

	double res = (1 - params.ALFA) * std::min(colourDist, params.TAU_COL) +
			params.ALFA * std::min(gradientDist, params.TAU_GRAD);
	
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
		throw std::domain_error("adaptiveWeight(). Out of bounds (" +
				to_string(w1) + ", " + to_string(h1) + ")");
	}
	if (w2 >= width || h2 >= height) {
		throw std::domain_error("adaptiveWeight(). Out of bounds (" +
				to_string(w2) + ", " + to_string(h2) + ")");
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

	return std::exp(-colourDist/params.GAMMA);
}


/****************************************************************************
* > pixelWindowCost()                                                       *
* Computes the total matching cost for pixel (w,h). The total cost is the   *
* sum of all matching costs for each pixel in a square window around (w,g). *
* Each pixel in the window is matched againts a pixel in the other view,    *
* according to the disparity function 'disparity'.                          *
* NOTE: Out of bounds pixels are ignored.                                   *
*                                                                           *
* Args:                                                                     *
*   w (size_t): width coordinate of the pixel                               *
*   h (size_t): height coordinate of the pixel                              *
*   disparity (PlaneFunction): disparity plane                              *
*                                                                           *
* Returns:                                                                  *
*   (double): window matching cost                                          *
****************************************************************************/
double StereoImage::pixelWindowCost(size_t w, size_t h,
		const PlaneFunction& disparity) const {
	
	// checks
	if (w >= width || h >= height) {
		throw std::domain_error("pixelWindowCost(). Out of bounds (" +
				to_string(w) + ", " + to_string(h) + ")");
	}

	// Setting the lenght of the window
	unsigned halfSideW = params.WINDOW_SIZE / 2;
	unsigned halfSideH = params.WINDOW_SIZE / 2;

	// Shrink slanted windows?
	if (params.RESIZE_WINDOWS) {
		double ncx = disparity.getParams().first(0);  // directional cosine of the
		double ncy = disparity.getParams().first(1);  // normal
		double cx = std::sqrt(1 - ncx * ncx);   // directional cosine of the plane
		double cy = std::sqrt(1 - ncy * ncy);

		halfSideW = std::round(params.WINDOW_SIZE * cx) / 2;
		halfSideH = std::round(params.WINDOW_SIZE * cy) / 2;
	}

	// Setting the extremes of the window
	size_t minW = (w > halfSideW) ? (w - halfSideW) : 0;
	size_t maxW = (w + halfSideW >= width) ? (width - 1) : (w + halfSideW);
	size_t minH = (h > halfSideH) ? (h - halfSideH) : 0;
	size_t maxH = (h + halfSideH >= height) ? (height - 1) : (h + halfSideH);

	// Scan each pixel in the window
	double totalCost = 0;
	int nPixels = 0;
	for (size_t iW = minW; iW <= maxW; ++iW) {
		for (size_t iH = minH; iH <= maxH; ++iH) {

			// Is this a valid cost?
			double dissimilarity = pixelDissimilarity(iW, iH, disparity);
			if (std::isnan(dissimilarity)) { continue; }

			// Accumulate the total with the current pixel
			totalCost += adaptiveWeight(w, h, iW, iH) * dissimilarity;
			++nPixels;
		}
	}

	// Normalizing the cost
	double total = totalCost / nPixels;

	// Check Nan cost due to borders
	if (nPixels == 0) {
		if (((side == LEFT && w > (unsigned)params.MAX_D) ||		// positive MAX_D
				(side == RIGHT && w < (width - params.MAX_D))) &&
				params.PLANES_SATURATION) { // this is an error only if we
			                              // always saturate
			throw std::logic_error("pixelWindowCost(). The window of pixel (" +
					to_string(w) + ", " + to_string(h) +
					") shouldn't fall completely outside\n" +
					"Plane: " + sStr(disparityPlanes.get(w, h)));
		} else {
			return 300; // NOTE: pixel on the border: can't compute disparity
		}
	}
	
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
		throw std::logic_error("bind(). Bad pointer");
	}
	if (other != nullptr || o->other != nullptr) {
		throw std::logic_error("bind(). An instance is bound already");
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
		throw std::logic_error("unbind(). Unbound already");
	}

	// unbind
	other->other = nullptr;
	other = nullptr;
}


/************************************************************************
* > setRandomDisparities()                                              *
* Set all 'disparityPlanes' to random linear functions. The range of    *
* disparity values in the central pixel of each plane is given by the   *
* parameters [params.MIN_D, params.MAX_D]. The angle of the plane is at *
* most params.MAX_SLOPE.                                                *
************************************************************************/
void StereoImage::setRandomDisparities(void) {

	for (size_t w = 0; w < width; ++w) {
		for (size_t h = 0; h < height; ++h) {
			disparityPlanes(w, h).setRandomFunction(w, h,
					params.MIN_D, params.MAX_D, 0, params.MAX_SLOPE);

			// Force planar windows if requested
			if (params.CONST_DISPARITIES) {
				disparityPlanes(w, h).setPlane({0,0,-1}, disparityPlanes(w, h)(w, h));
			}
		}
	}
}


/****************************************************************************
* > pixelSpatialPropagation()                                               *
* Spatial propagation step for a single pixel. If the plane of a spatial    *
* neighbor of p = (w,h) has a lower cost than the current plane of p, that  *
* plane is assigned to p. If iteration is an even number, the neighbors are *
* the left and upper pixels.                                                *
* NOTE: bounds are not checked                                              *
*                                                                           *
* Args:                                                                     *
*   w (size_t), h (size_t): coordinates of the pixel to check               *
*   iteration (unsigned): the iteration number                              *
*                                                                           *
* Returns:                                                                  *
*   (bool): true if the plane of p has been modified                        *
****************************************************************************/
bool StereoImage::pixelSpatialPropagation(size_t w, size_t h,
		unsigned iteration) {

	// Direction
	int direction = (iteration % 2 == 0) ? -1 : 1;

	// Do the neighbors exist?
	bool hasHorizontalPixel = (direction == -1) ? (w > 0) : (w < width-1);
	bool hasVerticalPixel = (direction == -1) ? (h > 0) : (h < height-1);

	// Neighbors
	size_t horzW = w + direction;  // modular arithmetic
	size_t vertH = h + direction;

	// 
	bool modified = false;
	double thisCost = pixelWindowCost(w, h, disparityPlanes.get(w, h));

	// Horizontal
	if (hasHorizontalPixel) {
		auto&& horzPlane = disparityPlanes.get(horzW, h);
		double horzCost = pixelWindowCost(w, h, horzPlane);
		if (horzCost < thisCost) {
			disparityPlanes(w, h) = horzPlane;
			modified = true;
		}
	}

	// Vertical
	if (hasVerticalPixel) {
		auto&& vertPlane = disparityPlanes.get(w, vertH);
		double vertCost = pixelWindowCost(w, h, vertPlane);
		if (vertCost < thisCost) {
			disparityPlanes(w, h) = vertPlane;
			modified = true;
		}
	}

	return modified;
}


/*****************************************************************************
* > pixelViewPropagation                                                     *
* View propagation step for a single pixel. It checks whether some pixels in *
* the other view have the current pixel p as a matching point. If any of     *
* those pixels' planes have lower cost with p, that plane is assigned to p.  *
* NOTE: the cost is proportional to the image width                          *
* NOTE: the plane of the other view is not transformed into this view        *
* NOTE: bounds for (w,h) are not checked                                     *
*                                                                            *
* Args:                                                                      *
*   w (size_t), h (size_t): coordinates of the pixel to check                *
*                                                                            *
* Returns:                                                                   *
*   (bool): true if the plane of (w,h) has been modified                     *
*****************************************************************************/
bool StereoImage::pixelViewPropagation(size_t w, size_t h) {

	// check
	if (other == nullptr) {
		throw std::logic_error("pixelViewPropagation(). Instance not bound");
	}

	bool modified = false;

	// Scan the horizontal (epipolar) line
	for (size_t oW = 0; oW < width; ++oW) {

		// Other plane disparity
		auto&& otherPlane = other->disparityPlanes.get(oW, h);
		int oDisparity = std::lround(otherPlane(oW, h));
		
		if (params.PLANES_SATURATION) { // Plane saturation on/off
			if (oDisparity > params.MAX_D) oDisparity = params.MAX_D;
			if (oDisparity < params.MIN_D) oDisparity = params.MIN_D;
		}

		// Find a plane that matches the current pixel
		int sign = (other->side == Side::LEFT) ? -1 : +1;
		size_t oDispW = oW + sign * oDisparity;
		if (oDispW == w) {

			// Test plane
			double otherCost = pixelWindowCost(w, h, otherPlane);
			double thisCost = pixelWindowCost(w, h, disparityPlanes.get(w, h));
			if (otherCost < thisCost) {
				disparityPlanes(w, h) = otherPlane;
				modified = true;
			}
		}
	}

	return modified;
}


/******************************************************************************
* > planeRefinement()                                                         *
* This method tries new random planes for pixel (w,h). Initially the plane    *
* may be far from the original one, but in the next iterations the            *
* modifications become smaller and smaller. At each step, a plane is accepted *
* only if the new one has a lower cost than the previous one.                 *
* NOTE: bounds for (w,h) are not checked.                                     *
* Important NOTE: the cost of this operation is variable.                     *
* The number of planes tested follows a geometric distribution                *
* with probability ~ 0.45 in the worst case.                                  *
*                                                                             *
* Args:                                                                       *
*   w (size_t), h (size_t): coordinates of the plane to refine                *
*                                                                             *
* Returns:                                                                    *
*   (bool): true if the plane has changed                                     *
******************************************************************************/
bool StereoImage::planeRefinement(size_t w, size_t h) {
	
	bool modified = false;
	
	// Set the initial ranges. Initally, the maximum variation.
	PlaneFunction& plane = disparityPlanes(w, h);
	double deltaZ = (params.MAX_D - params.MIN_D) / 2;
	double deltaAng = 89;		// < 90° in any direction
	
	double thisCost;
	bool recomputeThisCost = true;  // for efficiency

	// Try different planes. Stop with a threshold
	while (deltaZ > 0.1) {

		// Compute the range for Z
		double z = disparityAt(w, h);
		double maxZ = z + deltaZ;
		double minZ = z - deltaZ;
		if (maxZ > params.MAX_D) { maxZ = params.MAX_D; }
		if (minZ < params.MIN_D) { minZ = params.MIN_D; }

		// Sample a new plane until we get a slope in the desired range
		PlaneFunction sampledPlane;
		do {
			sampledPlane = plane.getNeighbourFunction(w, h, minZ, maxZ, deltaAng);
		} while (std::abs(sampledPlane.getParams().first(2)) <
				std::cos(params.MAX_SLOPE));

		// Set if the new one has lower cost
		if (recomputeThisCost) {
			thisCost = pixelWindowCost(w, h, disparityPlanes.get(w, h));
			recomputeThisCost = false;
		}
		double sampledCost = pixelWindowCost(w, h, sampledPlane);
		if (sampledCost < thisCost) {
			plane = sampledPlane;  // this is a ref: modifies disparityPlanes
			modified = true;
			recomputeThisCost = true;
		}

		// Half range
		deltaZ /= 2;
		deltaAng /= 2;
	}
	
	return modified;
}


/************************************************************************
* > getInvalidPixelsMap()                                               *
* Returns a black/white image with the invalidated pixels in black.     *
* Invalidate pixels (or planes) are the coordinates in which left/right *
* disparities do not match (differ by > 1).                             *
*                                                                       *
* Returns:                                                              *
*   (Image): a map of the invalidated pixels for this image             *
************************************************************************/
Image StereoImage::getInvalidPixelsMap(void) const {

	// checks
	if (other == nullptr) {
		throw std::logic_error("getInvalidPixelsMap(). Instance not bound");
	}

	// Initialize the map with all valid pixels
	Image invalidMap(width, height, 1, 255);

	// Get the disparities
	Image disparity = getDisparityMap();
	Image oDisparity = other->getDisparityMap();

	// Iterate on this image
	for (size_t w = 0; w < width; ++w) {
		for (size_t h = 0; h < height; ++h) {

			// Coordinates of the matching pixel
			int sign;
			switch (side) {
				case LEFT: sign = -1; break;
				case RIGHT: sign = +1; break;
			}
			double d = disparity.get(w,h);
			double oWD = w + sign * d;
			if (oWD < 0 || oWD >= other->width) {  // If it is projected outside
				invalidMap(w,h) = 0;
				continue;
			}
			size_t oW = std::lround(oWD);

			// Compare the other pixel dispatity
			double oD = oDisparity.get(oW,h);
			if (std::abs(d - oD) > 1) {
				invalidMap(w,h) = 0;
			}
		}
	}

	return invalidMap;
}


/*****************************************************************************
* > fillInvalidPlanes()                                                      *
* Computes the set of invalid planes through consistency checks on the       *
* left/right pair. Then updates the plane of invalidated pixels. The chosen  *
* plane is the one having lower disparity among the left/right valid pixels. *
* (Background fill)                                                          *
*****************************************************************************/
void StereoImage::fillInvalidPlanes(void) {

	// Mark the invalid planes as black
	Image valid = getInvalidPixelsMap();

	// LEFT view is filled right to left (due to errors in the left band)
	// RIGHT view is filled left to right (due to errors in the left band)
	int increment = (side == LEFT) ? -1 : 1;
	size_t wFirst = (increment > 0) ? 0 : (width - 1);
	size_t wLast = (increment > 0) ? (width - 1) : 0;

	// Fill by line
	for (size_t h = 0; h < height; ++h) {

		// Save the last and next valid pixels (can't use left and right)
		size_t wValidL = wFirst, wValidN = wFirst;
		bool hasLast = false, hasNext = true;

		// Scan each line
		for (size_t w = wFirst; true; w += increment) {

			// Should I fill this?
			if (!valid(w,h)) {

				// Find the next valid, if necessary and if has one
				if (((increment > 0) ? (w >= wValidN) : (w <= wValidN)) && hasNext) {
					for (size_t wI = w; true; wI += increment) {
						if (valid(wI,h)) {
							wValidN = wI;
							break; // found
						}
						if (wI == wLast) {
							hasNext = false;
							break;
						}
					}
				}

				// Compute the disparity of (w,h) with both planes and
				// assign the lower one
				if (hasLast) {
					disparityPlanes(w, h) = disparityPlanes.get(wValidL, h);
				}
				if (hasNext) {
					auto& currentPlane = disparityPlanes.get(w,h);
					auto& nextPlane = disparityPlanes.get(wValidN,h);
					if ((currentPlane(w,h) > nextPlane(w,h)) || (!hasLast)) {
						disparityPlanes(w,h) = nextPlane;
					}
				}

			} else {
				wValidL = w;
				hasLast = true;
			}

			if (w == wLast) break;
		}
	}
}


// > class StereoImagePair

/******************************************************
* > StereoImagePair()                                 *
* Constructor.                                        *
*                                                     *
* Args:                                               *
*   leftImgPath (string): path of the left RGB view   *
*   rightImgPath (string): path of the right RGB view *
******************************************************/
StereoImagePair::StereoImagePair(const string& leftImgPath,
		const string& rightImgPath):
		leftImg(leftImgPath, StereoImage::LEFT),
		rightImg(rightImgPath, StereoImage::RIGHT),
		width(leftImg.size().first),
		height(leftImg.size().second) {
		
	leftImg.bind(&rightImg);
}


/************************************************************************
* > computeDisparity()                                                  *
* Computes the disparity map of the two images using the PatchMatch     *
* Stereo algorithm. In even iterations we proceed left to right, top to *
* bottom; in odd iterations, in the opposite direction. See the         *
* reference paper for more.                                             *
*                                                                       *
* Returns:                                                              *
*   (pair<Image,Image>): the left and right disparity maps              *
************************************************************************/
pair<Image,Image> StereoImagePair::computeDisparity(void) {

	// Random Initialization
	leftImg.setRandomDisparities();
	rightImg.setRandomDisparities();
	logMsg("Random Initialization done", 1);

	// For each iteration
	for (unsigned i = 0; i < params.ITERATIONS; ++i) {
		logMsg("Iteration #"+to_string(i+1), 1);

		// Select the direction
		int increment = (i % 2 == 0) ? 1 : -1;
		size_t wFirst, wLast, hFirst, hLast;
		if (increment > 0) {  // start from 0
			wFirst = 0;
			hFirst = 0;
			wLast = width - 1;
			hLast = height - 1;
		} else {   // start from the end
			wFirst = width - 1;
			hFirst = height - 1;
			wLast = 0;
			hLast = 0;
		}

		// For each of the two images
		auto image = &leftImg;
		for (unsigned v = 0; v < 2; ++v, image = &rightImg) {
			logMsg(string("Processing the ") +
					((image == &leftImg) ? "left " : "right ") + "image" , 1);

			// For each pixel: row major order.
			//		NOTE: whole image, not ignoring lateral bands
			for (size_t h = hFirst; true; h += increment) {
				logMsg("(_,"+to_string(h)+")", 2, ' ', true);

				for (size_t w = wFirst; true; w += increment) {
					logMsg("("+to_string(w)+","+to_string(h)+")", 3, ' ');

					// Spatial propagation
					image->pixelSpatialPropagation(w, h, i);

					// View propagation
					image->pixelViewPropagation(w, h);
					
					// Plane refinement
					image->planeRefinement(w, h);

					if (w == wLast) break;
				}
				if (h == hLast) break;
			}
			logMsg("", 2);
		}
	}

	// Post processing TODO: add weighted median filter
	logMsg("Post processing" , 1);
	leftImg.fillInvalidPlanes();
	rightImg.fillInvalidPlanes();

	// Return both disparity maps
	return {leftImg.getDisparityMap(), rightImg.getDisparityMap()};
}
