/* Copyright 2014-2018 Rsyn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#ifndef RSYN_DOUBLE_RECTANGLE_H
#define RSYN_DOUBLE_RECTANGLE_H

#include <algorithm>
using std::max;
using std::min;
#include <limits>

#include "dim.h"
#include "double2.h"
#include "Bounds.h"

// -----------------------------------------------------------------------------

class DoubleRectangle {
	friend ostream &operator<<( ostream &out, const DoubleRectangle &r ) {
		return out << "[" << r[LOWER] << ", " << r[UPPER] << "]";
	} // end function

private:
	double2 clsPoints[2];
public:

	DoubleRectangle() {
		clsPoints[0] = double2();
		clsPoints[1] = double2();
	}

	DoubleRectangle(const double xmin, const double ymin, const double xmax, const double ymax ) {
		(*this)[LOWER].set(xmin,ymin);
		(*this)[UPPER].set(xmax,ymax);
	} // end constructor

	DoubleRectangle(const double2 pmin, const double2 pmax ) {
		(*this)[LOWER] = pmin;
		(*this)[UPPER] = pmax;
	} // end constructor

	DoubleRectangle(const Bounds &bounds) {
		operator=(bounds);
	} // end constructor

	void operator=(const Bounds &bounds) {
		clsPoints[0].x = bounds[0].x;
		clsPoints[0].y = bounds[0].y;
		clsPoints[1].x = bounds[1].x;
		clsPoints[1].y = bounds[1].y;
	} // end constructor

	Bounds scaleAndConvertToDbu(const double scale, 
			const RoundingStrategy roundingLower,
			const RoundingStrategy roundingUpper) const {
		DoubleRectangle copy = *this;
		copy.scaleCoordinates(scale);
		return copy.convertToDbu(roundingLower, roundingUpper);
	} // end method

	Bounds convertToDbu(
			const RoundingStrategy roundingLower,
			const RoundingStrategy roundingUpper) const {
		return Bounds(clsPoints[0].convertToDbu(roundingLower), clsPoints[1].convertToDbu(roundingUpper));
	} // end method

	Bounds scaleAndConvertToDbu(const double scale,
			const RoundingStrategy rounding = ROUND_DOWN) const {
		return scaleAndConvertToDbu(scale, rounding, rounding);
	} // end method

	Bounds convertToDbu(const RoundingStrategy rounding = ROUND_DOWN) const {
		return convertToDbu(rounding, rounding);
	} // end method

	void updatePoints(const double2 pmin, const double2 pmax){
		(*this)[LOWER] = pmin;
		(*this)[UPPER] = pmax;
	}
	void updatePoints(const double xMin, const double yMin, const double xMax, const double yMax){
		(*this)[LOWER][X] = xMin;
		(*this)[LOWER][Y] = yMin;
		(*this)[UPPER][X] = xMax;
		(*this)[UPPER][Y] = yMax;
	}
	double2 &operator[](const int boundary)       { return clsPoints[boundary]; }
	const double2 &operator[](const int boundary) const { return clsPoints[boundary]; }

	double computeLength(const int dimension) const { return (*this)[UPPER][dimension] - (*this)[LOWER][dimension]; }
	double2 computeLength() const { return (*this)[UPPER] - (*this)[LOWER]; }

	double computeDiagonal() const { return std::sqrt( std::pow(computeLength(X), 2.0) + std::pow(computeLength(Y), 2.0) ); }

	double computeCenter(const int dimension) const { return ( (*this)[UPPER][dimension] + (*this)[LOWER][dimension] ) / 2.0; }
	double2 computeCenter() const { return ( (*this)[UPPER] + (*this)[LOWER] ) / 2.0; }

	double computeArea() const { return computeLength(0)*computeLength(1); }
	double computeSemiperimeter() const { return computeLength(0)+computeLength(1); }

	double randomInnerPoint(const int dimension) const {
		const double random = rand()/double(RAND_MAX);
		return (*this)[LOWER][dimension] + random*computeLength(dimension);
	} // end method

	double overlapArea( const DoubleRectangle &rect ) const {
		const double dx = max( (*this)[LOWER][X], rect[LOWER][X] ) - min( (*this)[UPPER][X], rect[UPPER][X] );
		const double dy = max( (*this)[LOWER][Y], rect[LOWER][Y] ) - min( (*this)[UPPER][Y], rect[UPPER][Y] );

		return (dx<0 && dy<0) ? dx*dy : 0;
	} // end method

	double getCoordinate(const Boundary bound, const Dimension dim ) const { return (*this)[bound][dim]; }
	double2 getCoordinate(const Boundary bound ) const { return (*this)[bound]; }

	bool overlap( const DoubleRectangle &rect ) const {
		const double dx = max( (*this)[LOWER][X], rect[LOWER][X] ) - min( (*this)[UPPER][X], rect[UPPER][X] );
		const double dy = max( (*this)[LOWER][Y], rect[LOWER][Y] ) - min( (*this)[UPPER][Y], rect[UPPER][Y] );

		return (dx<0 && dy<0);
	} // end method

	// Returns the rectangle formed by the intersection of this and other
	// rectangle. If they don't overlap returns a degenerated rectangle (zero
	// area) at half way between the two rectangles.
	DoubleRectangle overlapRectangle( const DoubleRectangle &rect ) const {
		const double dx = max( (*this)[LOWER][X], rect[LOWER][X] ) - min( (*this)[UPPER][X], rect[UPPER][X] );
		const double dy = max( (*this)[LOWER][Y], rect[LOWER][Y] ) - min( (*this)[UPPER][Y], rect[UPPER][Y] );

		DoubleRectangle overlap;
		if ( dx<0 && dy<0 ) {
			overlap[LOWER] = max( (*this)[LOWER], rect[LOWER] );
			overlap[UPPER] = min( (*this)[UPPER], rect[UPPER] );
		} else {
			// Returns a zero-area rectangle at half the way of the two
			// rectangles.
			const double2 upper = max( (*this)[LOWER], rect[LOWER] );
			const double2 lower = min( (*this)[UPPER], rect[UPPER] );

			overlap[LOWER] = (upper+lower) / 2.0;
			overlap[UPPER] = overlap[LOWER];
		} // end else

		return overlap;
	} // end method

	// Check if a value is in between the lower and upper bounds of this
	// rectangle.
	bool between( const double pos, const Dimension DIMENSION ) const {
		return ( pos >= (*this)[LOWER][DIMENSION] && pos <= (*this)[UPPER][DIMENSION] );
	} // end method

	// Check if a point is inside this rectangle.

	bool inside( const double x, const double y ) const {
		return
			( x >= (*this)[LOWER][X] && x <= (*this)[UPPER][X] ) &&
			( y >= (*this)[LOWER][Y] && y <= (*this)[UPPER][Y] );
	} // end method

	bool inside( const double2 pos ) const {
		return between(pos[X],X) && between(pos[Y],Y);
	} // end method

	// Change the position of this rectangle.

	void moveTo( const double position, const Dimension DIMENSION ) {
		const double length = computeLength(DIMENSION);
		(*this)[LOWER][DIMENSION] = position;
		(*this)[UPPER][DIMENSION] = position + length;
	} // end method

	void moveTo( const double2 position ) {
		const double2 length = computeLength();
		(*this)[LOWER] = position;
		(*this)[UPPER] = position + length;
	} // end method

	void moveCenterTo( const double2 position ) {
		const double2 halfLength = computeLength() / 2.0;
		(*this)[LOWER] = position - halfLength;
		(*this)[UPPER] = position + halfLength;
	} // end method

	void translate( const double2 displacement ) {
		(*this)[LOWER] += displacement;
		(*this)[UPPER] += displacement;
	} // end method

	void moveTo( const double x, const double y ) { moveTo(double2(x,y)); }
	void moveCenterTo( const double x, const double y ) { moveCenterTo(double2(x,y)); }
	void translate( const double x, const double y ) { translate(double2(x,y)); }

	// Multiple each coordinates by a scale factor. Useful when change the
	// coordinate system.
	void scaleCoordinates(const double numb){
		clsPoints[LOWER].scale(numb);
		clsPoints[UPPER].scale(numb);
	} // end method

	// Scale this rectangle using its center as the point of reference. Note
	// that this will keep the center of the rectangle in the same position.

	void scaleCentralized(const double2 factor) {
		const double2 p = computeCenter();
		translate(-p);
		clsPoints[LOWER] *= factor;
		clsPoints[UPPER] *= factor;
		translate(+p);
	} // end method

	void scaleCentralized(const double factor) {
		scaleCentralized(double2(factor, factor));
	} // end method

	// Check if this rectangle is valid, that is the lower coordinates are
	// smaller than the upper coordinates.
	bool isValid() const {
		return
			(*this)[LOWER][X] <= (*this)[UPPER][X] &&
			(*this)[LOWER][Y] <= (*this)[UPPER][Y];
	} // end method

	// Get the point inside the rectangle which is closest to p.
	double2 closestPoint(const double2 p) const {
		double2 lower = getCoordinate(LOWER);
		double2 upper = getCoordinate(UPPER);
		return max(min(p, upper), lower);
	} // end method

	// Make the lower be +inf and upper be -inf. Useful when computing the
	// bounding box of a set of points.
	void degenerate() {
		(*this)[LOWER][X] = +std::numeric_limits<double>::infinity();
		(*this)[LOWER][Y] = +std::numeric_limits<double>::infinity();
		(*this)[UPPER][X] = -std::numeric_limits<double>::infinity();
		(*this)[UPPER][Y] = -std::numeric_limits<double>::infinity();
	} // end method

	// Increases this rectangle so that the point x, y will be inside it.

	void stretchToFit(const double x, const double y) {
		(*this)[LOWER] = min((*this)[LOWER], double2(x, y));
		(*this)[UPPER] = max((*this)[UPPER], double2(x, y));
	} // end method

	void stretchToFit(const double2 p) {
		stretchToFit(p.x, p.y);
	} // end method

	void clear () {
		clsPoints[0].clear();
		clsPoints[1].clear();
	} // end method

}; // end struct


#endif

