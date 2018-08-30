
#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Core>


using Eigen::Vector3d;
using std::pair;


/*****************************************************************************
* > class Plane                                                              *
* The representation of a plane in 3D spaces. See the comments in .cpp file. *
*****************************************************************************/
class Plane {
	
	private:

		// plane coefficients
		Vector3d abc;
		double d;

	public:

		// construct
		//Plane(void): abc(1,0,0), d(0) {}
		Plane(double a, double b, double c, double d);

		// const methods
		pair<Vector3d, double> getParams(void) const { return {abc, d}; }
		double distanceOfPoint(const Vector3d& point) const;
		pair<Vector3d, Vector3d> toPointAndNorm(void) const;

		// other methods
		Plane& setPlane(const Vector3d& abc, double d);
		Plane& fromPointAndNorm(const Vector3d& point, const Vector3d& norm);

		// operator
		friend std::ostream& operator<<(std::ostream& out, const Plane& p);

};
