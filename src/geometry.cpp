
#include "geometry.hpp"
#include "utils.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <random>
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::AngleAxisd;


// > class Plane


/**************************************************************************
* > Plane()                                                               *
* Constructs from the 4 parameters. Internally, the parameters are scaled *
* so that a,b,c creates an unit vector.                                   *
*                                                                         *
* Args:                                                                   *
*   a (double), b (double), c (double), d (double): the four parameters   *
*     of the plane.                                                       *
**************************************************************************/
Plane::Plane(double a, double b, double c, double d):
		abc(a,b,c), d(d) {
	
	// Assuming this is a valid equation: at least one != 0
	double len = abc.norm();
	this->abc /= len;
	this->d /= len;
}


/**************************************************************************
* > Plane()                                                               *
* Constructs from the 4 parameters. Internally, the parameters are scaled *
* so that a,b,c creates an unit vector.                                   *
*                                                                         *
* Args:                                                                   *
*   abc (Vector3d): the first three parameters                            *
*   d (double): the fourth parameter                                      *
**************************************************************************/
Plane::Plane(const Vector3d abc, double d):
		Plane(abc(0), abc(1), abc(2), d) {}


/*******************************************************************
* > setPlane()                                                     *
* Set the current plane with the given parameters. Parameters will *
* be scaled.                                                       *
*                                                                  *
* Args:                                                            *
*   abc (Vector3d&): the abc components.                           *
*   d (double): the d component.                                   *
*                                                                  *
* Returns:                                                         *
*   (Plane&): reference to this.                                   *
*******************************************************************/
Plane& Plane::setPlane(const Vector3d& abc, double d) {
	double len = abc.norm();
	this->abc = abc/len;
	this->d = d/len;

	return *this;
}


/*************************************************************************
* > setRandomPlane()                                                     *
* Sets the current plane as random in a range with uniform distribution. *
* The range is the position of the plane with respect to the origin      *
* (distance with sign).                                                  *
*                                                                        *
* Args:                                                                  *
*   double (d1): minimum distance (with sign)                            *
*   double (d2): maximum distance (with sign)                            *
*                                                                        *
* Returns:                                                               *
*   (Plane&): reference to this                                          *
*************************************************************************/
Plane& Plane::setRandomPlane(double d1, double d2) {

	// Normal
	abc = randomNormal();

	// When the normal is an unit vector,
	// d is just the distance of the origin (with sign)
	auto& rand = RandomDevice::getGenerator();
	std::uniform_real_distribution<double> uniform(d1, d2);
	d = uniform(rand.engine);

	return *this;
}


/******************************************************************
* > fromPointAndNorm()                                            *
* Set the plane parameters from the point and norm representation *
*                                                                 *
* Args:                                                           *
*   point (Vector3d&): a point of the plane                       *
*   norm (Vector3d&): the normal of the plane                     *
* Return:                                                         *
*   (Plane&): reference to this                                   *
******************************************************************/
Plane& Plane::fromPointAndNorm(const Vector3d& point, const Vector3d& norm) {

	abc = norm.normalized();
	d = - abc.dot(point);

	return *this;
}


/************************************************************************
* > toPointAndNorm()                                                    *
* Converts the plane coefficients to the point and norm representation. *
* The chosen point will be the closest to the origin.                   *
*                                                                       *
* Returns:                                                              *
*   (tuple{point, norm}): the two Vector3d vectors.                     *
************************************************************************/
pair<Vector3d, Vector3d> Plane::toPointAndNorm(void) const {

	Vector3d point = -abc*d; // NOTE: assuming abc are an unit norm
	return {point, abc};
}


/*********************************************************
* > distanceOfPoint()                                    *
* Computes the distance of the given point to the plane. *
*                                                        *
* Args:                                                  *
*   point (Vector3d&): any point in space                *
* Returns:                                               *
*   (double): the distance of that point to the plane    *
*********************************************************/
double Plane::distanceOfPoint(const Vector3d& point) const {
	
	// NOTE: Assuming that abc is a unit vector
	return std::abs(abc.dot(point) + d);
}


// print
std::ostream& operator<<(std::ostream& out, const Plane& p) {
	return out << "{ " << p.abc.transpose() << "  " << p.d << "}";
}


/****************************************************************
* > randomNormal()                                              *
* Samples a point on the unit sphere with uniform distribution. *
*                                                               *
* Returns:                                                      *
*   (Vector3d): the random unit vector                          *
****************************************************************/
Vector3d Plane::randomNormal(void) const {

	// The 3d unit normal distribution uniformly samples
	// directions on the sphere
	std::normal_distribution<double> normalDist;
	auto& rand = RandomDevice::getGenerator();

	Vector3d v { normalDist(rand.engine),
			normalDist(rand.engine),
			normalDist(rand.engine) };
	v.normalize();

	return v;
}


// > class PlaneFunction

const double PlaneFunction::Z_EPS = 0.01; 


/***************************************************************************
* > PlaneFunction()                                                        *
* Constructs a linear function in two arguments with the given parameters. *
* The interpretation is: a*x + b*y + c. c is a constant here.              *
*                                                                          *
* Args:                                                                    *
*   a (double), b (double), c (double): the three parameters               *
***************************************************************************/
PlaneFunction::PlaneFunction(double a, double b, double c):
		Plane(a,b,-1,c) {}


/*****************************************************************************
* > getFunParams()                                                           *
* Returns:                                                                   *
*   (Vector3d): the parameters with the function notation (slope x, slope y, *
*       constant).                                                           *
*****************************************************************************/
Vector3d PlaneFunction::getFunParams() const {
	double c = abc(2);   // NOTE: assuming  c != 0
	return { -abc(0)/c, -abc(1)/c, -d/c };
}


/********************************************************************
* > setPlane                                                        *
* Sets the plane representation, given the four parameters.         *
* If not a function, throws a runtime_error.                        *
*                                                                   *
* Args:                                                             *
*   abc (Vector3d): the first three parameters. abc(2) must be != 0 *
*   d (double): the fourth parameter                                *
*                                                                   *
* Returns:                                                          *
*   (PlaneFunction&): reference to this                             *
********************************************************************/
PlaneFunction& PlaneFunction::setPlane(const Vector3d& abc, double d) {

	Plane::setPlane(abc, d);
	if (!areFunctionParams(abc)) {
		throw std::runtime_error("Plane coefficient for z < Z_EPS (in norm): " +
				std::to_string(abc(2)));
	}

	return *this;
}


/****************************************************************************
* > setRandomPlane()                                                        *
* Sets the current function as random in a range with uniform distribution. *
* The range is the position of the plane with respect to the origin         *
* (distance with sign).                                                     *
*                                                                           *
* Args:                                                                     *
*   d1 (double): minimum distance (with sign)                               *
*   d2 (double): maximum distance (with sign)                               *
*                                                                           *
* Returns:                                                                  *
*   (Plane&): reference to this                                             *
****************************************************************************/
PlaneFunction& PlaneFunction::setRandomPlane(double d1, double d2) {

	// Normal
	do {
		abc = randomNormal();
	} while (!areFunctionParams(abc));

	// When the normal is an unit vector,
	// d is just the distance of the origin (with sign)
	std::uniform_real_distribution<double> uniform(d1, d2);
	auto& rand = RandomDevice::getGenerator();
	d = uniform(rand.engine);

	return *this;
}


/***************************************************************************
* > setRandomFunction()                                                    *
* Sets the current function to be random in a range with uniform           *
* distribution. The range is the set of allowed values at the given point, *
* and the set of allowed angle of the plane with respect to the horizontal *
* plane.                                                                   *
*                                                                          *
* Args:                                                                    *
*   x (double): point x coord                                              *
*   y (double): point y coord                                              *
*   min (double): min output value at (x,y)                                *
*   max (double): max output value at (x,y)                                *
*   minAngle (double): the minimul slope. Expressed in degrees, in [0,90). *
*   maxAngle (double): the maximum slope. Expressed in degrees, in (0,90]. *
*                                                                          *
* Returns:                                                                 *
*   (PlaneFunction&): reference to this                                    *
***************************************************************************/
PlaneFunction& PlaneFunction::setRandomFunction(double x, double y,
		double min, double max, double minAngle, double maxAngle) {

	// Check
	if (maxAngle > 90 || maxAngle <= 0) {
		throw std::runtime_error(std::to_string(maxAngle) + " is not a valid " + 
				"maximum plane angle for setRandomFunction()");
	}
	
	// Sampling in the point--normal representation
	
	// point
	std::uniform_real_distribution<double> uniformZVal(min, max);
	auto& rand = RandomDevice::getGenerator();
	double zValue = uniformZVal(rand.engine);

	// normal
	double minZ = std::cos(maxAngle/180*M_PI);
	double maxZ = std::cos(minAngle/180*M_PI);
	std::uniform_real_distribution<double> uniform01;
	std::uniform_real_distribution<double> uniformZNorm(minZ, maxZ);
	double nZ = uniformZNorm(rand.engine); // uniform z component
	double phi = 2*M_PI* uniform01(rand.engine) - M_PI;
			// ^ uniform angle in [-pi,pi] in the x,y plane
	double nX = std::cos(phi) * std::sqrt(1 - nZ*nZ); // map to uniform x
	double nY = std::sin(phi) * std::sqrt(1 - nZ*nZ); // map to uniform y

	// Set
	fromPointAndNorm({x,y,zValue}, {nX,nY,nZ});

	return *this;
}


/*************************************************************************
* > setNeighbourFunction()                                               *
* Returns a plane as a random function in a neighbourhood of the         *
* given one. "In a neighbourhood" means that: its value at point         *
* (x,y) will be at most 'deltaZ' far from the current value; the normal  *
* will be within in a circular region on the unit sphere, around the     *
* current normal. Both are sampled with uniform distribution.            *
* The current values are taken form 'this' object.                       *
*                                                                        *
* Args:                                                                  *
*   x (double): first coordinate of a point                              *
*   y (double): second coordinate of a point                             *
*   deltaZ (double): max abs difference of the new z-value at (x,y)      *
*   deltaAng (double): max angle between the new normal and the old one, *
*       in (0,90). This is the angle of the circular region.             *
*                                                                        *
* Returns:                                                               *
*   (PlaneFunction): The new plane                                       *
*************************************************************************/
PlaneFunction PlaneFunction::getNeighbourFunction(double x, double y,
		double deltaZ, double deltaAng) const {

	double oldZ = operator()(x, y);

	return getNeighbourFunction(x, y, oldZ - deltaZ, oldZ + deltaZ,
			deltaAng);
}


/*****************************************************************************
* > getNeighbourFunction()                                                   *
* Returns a plane as a random function in a neighbourhood of 'this'          *
* plane. "In a neighbourhood" means that the normal is uniformly sampled     *
* in a circular region on the unit sphere around 'this' normal.              *
* The Z value instead is choosen in the range [zMin, zMax].                  *
*                                                                            *
* Args:                                                                      *
*   x (double): first coordinate of a point                                  *
*   y (double): second coordinate of a point                                 *
*   minZ (double): minimum z-value at (x,y)                                  *
*   maxZ (double): maximum z-value at (x,y)                                  *
*   deltaAng (double): max angle between the new normal and the current one, *
*       in (0,90). This is the angle of the circular region.                 *
*                                                                            *
* Returns:                                                                   *
*   (PlaneFunction): the new plane                                           *
*****************************************************************************/
PlaneFunction PlaneFunction::getNeighbourFunction(double x, double y,
		double minZ, double maxZ, double deltaAng) const {

	// checks
	if (maxZ <= minZ) {
		throw std::runtime_error("[ " + std::to_string(minZ) +
				", " + std::to_string(maxZ) + "] is not a valid Z range");
	}
	if (deltaAng <= 0 || deltaAng >= 90) {
		throw std::runtime_error(string() +
				"deltaT parameter of setNeighbourFunction() " +
				"must be in the range (0,90).");
	}

	// Get the representation of the old plane at (x,y)
	Vector3d oldNormal = abc;
	Vector3d oldPoint = {x, y, operator()(x, y)};

	// Get the spherical coordinates of oldNormal
	double oldTheta = std::acos(oldNormal(2));
	double oldPhi = std::atan2(oldNormal(1), oldNormal(0));

	Vector3d sampledNormal;
	double sampledZ;
	PlaneFunction newFun;
	do {  // Rarely repeats (with Z_EPS small enough)

		// Sampling as if in the oldNormal reference frame
		newFun.setRandomFunction(x, y, minZ, maxZ, 0, deltaAng);
		sampledNormal = newFun.abc;
		sampledZ = newFun(x, y);

		// Rotating in the global frame:  Rz(phi) * Ry(theta)
		Matrix3d rot;
		rot = AngleAxisd(oldPhi, Vector3d::UnitZ()) *
				AngleAxisd(oldTheta, Vector3d::UnitY());
		sampledNormal = rot * sampledNormal;

	} while (!areFunctionParams(sampledNormal));  

	// Set
	newFun.fromPointAndNorm({x, y, sampledZ}, sampledNormal);

	return newFun;
}


/*******************************************************************
* > fromPointAndNorm()                                             *
* Set the plane parameters from the point and norm representation. *
* If not a function (c==0) throws a runtime_error.                 *
*                                                                  *
* Args:                                                            *
*   point (Vector3d&): a point of the plane                        *
*   norm (Vector3d&): the normal of the plane                      *
* Return:                                                          *
*   (Plane&): reference to this                                    *
*******************************************************************/
PlaneFunction& PlaneFunction::fromPointAndNorm(const Vector3d& point,
		const Vector3d& norm) {

	Plane::fromPointAndNorm(point, norm);
	if (!areFunctionParams(abc)) {
		throw std::runtime_error("Plane coefficient for z < Z_EPS (in norm): " +
				std::to_string(abc(2)));
	}

	return *this;

}


/*******************************************************************************
* > operator()()                                                               *
* Function call operator. Evaluates the function at the given x,y coordinates. *
*                                                                              *
* Args:                                                                        *
*   x (double): first coordinate                                               *
*   y (double): second coordinate                                              *
*                                                                              *
* Returns:                                                                     *
*   (double): the z value                                                      *
*******************************************************************************/
double PlaneFunction::operator()(double x, double y) const {

	Vector3d coeffs(getFunParams());
	Vector3d point = { x, y, 1};

	return coeffs.dot(point);
}


// print
std::ostream& operator<<(std::ostream& out, const PlaneFunction& p) {
	return out << "{ " << p.getFunParams().transpose() << " }";
}


/*******************************************************
* > areFunctionParams()                                *
* Returns:                                             *
*   (bool): true if p is a valid normal for a function *
*******************************************************/
inline bool PlaneFunction::areFunctionParams(Vector3d p) const {
	return std::abs(p(2)) > Z_EPS;
}
