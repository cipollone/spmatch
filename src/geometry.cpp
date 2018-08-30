
#include "geometry.hpp"


// > class Plane

RandomDevice Plane::rndDev;

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
Vector3d Plane::randomNormal(void) {

	// The 3d unit normal distribution uniformly samples
	// directions on the sphere
	std::normal_distribution<double> normalDist;

	Vector3d v { normalDist(rndDev.rndDev),
			normalDist(rndDev.rndDev),
			normalDist(rndDev.rndDev) };
	v.normalize();

	return v;
}


// > class PlaneFunction

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
	if (abc(2) < Z_EPS) {
		throw std::runtime_error("Plane coefficient for z close to 0: " +
				std::to_string(abc(2)));
	}

	return *this;
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
	if (abc(2) < Z_EPS) {
		throw std::runtime_error("Plane coefficient for z close to 0: " +
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
