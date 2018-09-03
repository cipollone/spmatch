
#pragma once

#include "utils.hpp"
#include "image.hpp"


/*************************************************************
* > namespace Params                                         *
* A container for constants used in the PatchMatch algorithm *
*************************************************************/
namespace Params {


}


/*********************************************************************
* > class StereoImage                                                *
* An image and its additional informations used for stereo matching. *
* The methods implements parts of the PatchMatch Stereo algorithm.   *
* Some methods requires this image to be bound to another one.       *
* See the comments in .cpp file                                      *
*********************************************************************/
class StereoImage: public Image {

	private:

		CImg<double> imgGradient;
		Grid<PlaneFunction> pixelPlanes;

		StereoImage* other = nullptr;

	public:

		// constr
		StereoImage(const string& imgPath);			// copy implicitly deleted

		// const methods
		double getDisparityAt(size_t w, size_t h) const;
		void displayGradient(void) const;

		// methods
		void bind(StereoImage* o);
		void unbind(void);
};


/************************************************************************
* > class StereoImagePair                                               *
* Represents a couple of StereoImages. The method computeDisparity()    *
* returns an disparity image computed with PatchMatch Stereo algorithm. *
* See .cpp file                                                         *
************************************************************************/
class StereoImagePair {
	
	private:

		StereoImage leftImg;
		StereoImage rightImg;

	public:

		// constr
		StereoImagePair(const string& leftImgPath, const string& rightImgPath):
				leftImg(leftImgPath), rightImg(rightImgPath) {}

		
		// methods
		void displayBoth(void);
		Image computeDisparity(void);

		// operators
		friend std::ostream& operator<<(std::ostream& o, const StereoImagePair& p);

};
