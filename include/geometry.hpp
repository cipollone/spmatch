
#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Core>

#include "utils.hpp"


using Eigen::Vector3d;
using std::pair;


/*****************************************************************************
* > class Plane                                                              *
* The representation of a plane in 3D spaces. See the comments in .cpp file. *
*****************************************************************************/
class Plane {
	
	protected:

		// plane coefficients
		Vector3d abc;
		double d;

		static RandomDevice rndDev;

	public:

		// construct
		Plane(void): abc(1,0,0), d(0) {}
		Plane(double a, double b, double c, double d);

		// const methods
		pair<Vector3d, double> getParams(void) const { return {abc, d}; }
		double distanceOfPoint(const Vector3d& point) const;
		pair<Vector3d, Vector3d> toPointAndNorm(void) const;

		// other methods
		virtual Plane& setPlane(const Vector3d& abc, double d);
		virtual Plane& fromPointAndNorm(const Vector3d& point, const Vector3d& norm);

		// operator
		friend std::ostream& operator<<(std::ostream& out, const Plane& p);

		// static
		static Vector3d randomNormal(void);

};


/*************************************************************************
* > class PlaneFunction                                                  *
* Linear function R^2 -> R. The plane is used as representation of       *
* the linear function. x and y are the parameters, z is taken as result. *
* NOTE: parameter c can't be 0.                                          *
*************************************************************************/
class PlaneFunction: public Plane {

	public:

		const double Z_EPS = 0.00001;

		// construct
		PlaneFunction(void): Plane(0,0,1,0) {}
		PlaneFunction(double a, double b, double c);

		// const methods
		Vector3d getFunParams(void) const;

		// other methods
		PlaneFunction& setPlane(const Vector3d& abc, double d) override;
		PlaneFunction& fromPointAndNorm(const Vector3d& point,
				const Vector3d& norm) override;
		
		// operator
		double operator()(double x, double y) const;
		friend std::ostream& operator<<(std::ostream& out, const PlaneFunction& p);
};
