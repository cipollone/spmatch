
#pragma once


/*********************************************************************
* > class Plane                                                      *
* The representation of the plane both as the geometric object in 3D *
* space and as a 2D numeric interpolation.                           *
*********************************************************************/
class Plane {
	
	private:

		// plane coefficients
		double a;
		double b;
		double c;


	public:

		Plane(double a, double b, double c):
				a(a), b(b), c(c) {}

		Plane(void): Plane(0,0,0) {}

};
