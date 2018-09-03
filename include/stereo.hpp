
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
*********************************************************************/
class StereoImage: public Image {

	private:

		Grid<PlaneFunction> pixelPlanes;

	public:

		// constr
		StereoImage(const string& imgPath);			// copy implicitly deleted

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
