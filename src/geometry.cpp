
#include "geometry.hpp"


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
