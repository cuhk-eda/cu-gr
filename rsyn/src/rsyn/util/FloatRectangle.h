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
 
#ifndef RSYN_FLOAT_RECTANGLE_H
#define RSYN_FLOAT_RECTANGLE_H

#include <algorithm>
using std::max;
using std::min;
#include <limits>

#include "dim.h"
#include "float2.h"
#include "Bounds.h"

// -----------------------------------------------------------------------------

class FloatRectangle {
	friend ostream &operator<<( ostream &out, const FloatRectangle &r ) {
		return out << "[" << r[LOWER] << ", " << r[UPPER] << "]";
	} // end function

private:
	float2 clsPoints[2];
public:

	FloatRectangle() {
		clsPoints[0] = float2();
		clsPoints[1] = float2();
	}

	FloatRectangle(const float xmin, const float ymin, const float xmax, const float ymax ) {
		(*this)[LOWER].set(xmin,ymin);
		(*this)[UPPER].set(xmax,ymax);
	} // end constructor

	FloatRectangle(const float2 pmin, const float2 pmax ) {
		(*this)[LOWER] = pmin;
		(*this)[UPPER] = pmax;
	} // end constructor

	FloatRectangle(const Bounds &bounds) {
		operator=(bounds);
	} // end constructor

	void operator=(const Bounds &bounds) {
		clsPoints[0].x = bounds[0].x;
		clsPoints[0].y = bounds[0].y;
		clsPoints[1].x = bounds[1].x;
		clsPoints[1].y = bounds[1].y;
	} // end constructor

	Bounds scaleAndConvertToDbu(const float scale,
			const RoundingStrategy roundingLower,
			const RoundingStrategy roundingUpper) const {
		FloatRectangle copy = *this;
		copy.scaleCoordinates(scale);
		return copy.convertToDbu(roundingLower, roundingUpper);
	} // end method

	Bounds convertToDbu(
			const RoundingStrategy roundingLower,
			const RoundingStrategy roundingUpper) const {
		return Bounds(clsPoints[0].convertToDbu(roundingLower), clsPoints[1].convertToDbu(roundingUpper));
	} // end method

	Bounds scaleAndConvertToDbu(const float scale,
			const RoundingStrategy rounding = ROUND_DOWN) const {
		return scaleAndConvertToDbu(scale, rounding, rounding);
	} // end method

	Bounds convertToDbu(const RoundingStrategy rounding = ROUND_DOWN) const {
		return convertToDbu(rounding, rounding);
	} // end method

	void updatePoints(const float2 pmin, const float2 pmax){
		(*this)[LOWER] = pmin;
		(*this)[UPPER] = pmax;
	}
	void updatePoints(const float xMin, const float yMin, const float xMax, const float yMax){
		(*this)[LOWER][X] = xMin;
		(*this)[LOWER][Y] = yMin;
		(*this)[UPPER][X] = xMax;
		(*this)[UPPER][Y] = yMax;
	}
	float2 &operator[](const int boundary)       { return clsPoints[boundary]; }
	const float2 &operator[](const int boundary) const { return clsPoints[boundary]; }

	float computeLength(const int dimension) const { return (*this)[UPPER][dimension] - (*this)[LOWER][dimension]; }
	float2 computeLength() const { return (*this)[UPPER] - (*this)[LOWER]; }

	float computeDiagonal() const { return std::sqrt( std::pow(computeLength(X), 2.0f) + std::pow(computeLength(Y), 2.0f) ); }

	float computeCenter(const int dimension) const { return ( (*this)[UPPER][dimension] + (*this)[LOWER][dimension] ) / 2.0f; }
	float2 computeCenter() const { return ( (*this)[UPPER] + (*this)[LOWER] ) / 2.0f; }

	float computeArea() const { return computeLength(0)*computeLength(1); }
	float computeSemiperimeter() const { return computeLength(0)+computeLength(1); }

	float randomInnerPoint(const int dimension) const {
		const float random = rand()/float(RAND_MAX);
		return (*this)[LOWER][dimension] + random*computeLength(dimension);
	} // end method

	float overlapArea( const FloatRectangle &rect ) const {
		const float dx = max( (*this)[LOWER][X], rect[LOWER][X] ) - min( (*this)[UPPER][X], rect[UPPER][X] );
		const float dy = max( (*this)[LOWER][Y], rect[LOWER][Y] ) - min( (*this)[UPPER][Y], rect[UPPER][Y] );

		return (dx<0 && dy<0) ? dx*dy : 0;
	} // end method

	float getCoordinate(const Boundary bound, const Dimension dim ) const { return (*this)[bound][dim]; }
	float2 getCoordinate(const Boundary bound ) const { return (*this)[bound]; }

	bool overlap( const FloatRectangle &rect ) const {
		const float dx = max( (*this)[LOWER][X], rect[LOWER][X] ) - min( (*this)[UPPER][X], rect[UPPER][X] );
		const float dy = max( (*this)[LOWER][Y], rect[LOWER][Y] ) - min( (*this)[UPPER][Y], rect[UPPER][Y] );

		return (dx<0 && dy<0);
	} // end method

	// Returns the rectangle formed by the intersection of this and other
	// rectangle. If they don't overlap returns a degenerated rectangle (zero
	// area) at half way between the two rectangles.
	FloatRectangle overlapRectangle( const FloatRectangle &rect ) const {
		const float dx = max( (*this)[LOWER][X], rect[LOWER][X] ) - min( (*this)[UPPER][X], rect[UPPER][X] );
		const float dy = max( (*this)[LOWER][Y], rect[LOWER][Y] ) - min( (*this)[UPPER][Y], rect[UPPER][Y] );

		FloatRectangle overlap;
		if ( dx<0 && dy<0 ) {
			overlap[LOWER] = max( (*this)[LOWER], rect[LOWER] );
			overlap[UPPER] = min( (*this)[UPPER], rect[UPPER] );
		} else {
			// Returns a zero-area rectangle at half the way of the two
			// rectangles.
			const float2 upper = max( (*this)[LOWER], rect[LOWER] );
			const float2 lower = min( (*this)[UPPER], rect[UPPER] );

			overlap[LOWER] = (upper+lower) / 2.0;
			overlap[UPPER] = overlap[LOWER];
		} // end else

		return overlap;
	} // end method

	// Check if a value is in between the lower and upper bounds of this
	// rectangle.
	bool between( const float pos, const Dimension DIMENSION ) const {
		return ( pos >= (*this)[LOWER][DIMENSION] && pos <= (*this)[UPPER][DIMENSION] );
	} // end method

	// Check if a point is inside this rectangle.

	bool inside( const float x, const float y ) const {
		return
			( x >= (*this)[LOWER][X] && x <= (*this)[UPPER][X] ) &&
			( y >= (*this)[LOWER][Y] && y <= (*this)[UPPER][Y] );
	} // end method

	bool inside( const float2 pos ) const {
		return between(pos[X],X) && between(pos[Y],Y);
	} // end method

	// Change the position of this rectangle.

	void moveTo( const float position, const Dimension DIMENSION ) {
		const float length = computeLength(DIMENSION);
		(*this)[LOWER][DIMENSION] = position;
		(*this)[UPPER][DIMENSION] = position + length;
	} // end method

	void moveTo( const float2 position ) {
		const float2 length = computeLength();
		(*this)[LOWER] = position;
		(*this)[UPPER] = position + length;
	} // end method

	void moveCenterTo( const float2 position ) {
		const float2 halfLength = computeLength() / 2.0;
		(*this)[LOWER] = position - halfLength;
		(*this)[UPPER] = position + halfLength;
	} // end method

	void translate( const float2 displacement ) {
		(*this)[LOWER] += displacement;
		(*this)[UPPER] += displacement;
	} // end method

	void moveTo( const float x, const float y ) { moveTo(float2(x,y)); }
	void moveCenterTo( const float x, const float y ) { moveCenterTo(float2(x,y)); }
	void translate( const float x, const float y ) { translate(float2(x,y)); }

	// Multiple each coordinates by a scale factor. Useful when change the
	// coordinate system.
	void scaleCoordinates(const float numb){
		clsPoints[LOWER].scale(numb);
		clsPoints[UPPER].scale(numb);
	} // end method

	// Scale this rectangle using its center as the point of reference. Note
	// that this will keep the center of the rectangle in the same position.

	void scaleCentralized(const float2 factor) {
		const float2 p = computeCenter();
		translate(-p);
		clsPoints[LOWER] *= factor;
		clsPoints[UPPER] *= factor;
		translate(+p);
	} // end method

	void scaleCentralized(const float factor) {
		scaleCentralized(float2(factor, factor));
	} // end method

	// Check if this rectangle is valid, that is the lower coordinates are
	// smaller than the upper coordinates.
	bool isValid() const {
		return
			(*this)[LOWER][X] <= (*this)[UPPER][X] &&
			(*this)[LOWER][Y] <= (*this)[UPPER][Y];
	} // end method

	// Get the point inside the rectangle which is closest to p.
	float2 closestPoint(const float2 p) const {
		float2 lower = getCoordinate(LOWER);
		float2 upper = getCoordinate(UPPER);
		return max(min(p, upper), lower);
	} // end method

	// Make the lower be +inf and upper be -inf. Useful when computing the
	// bounding box of a set of points.
	void degenerate() {
		(*this)[LOWER][X] = +std::numeric_limits<float>::infinity();
		(*this)[LOWER][Y] = +std::numeric_limits<float>::infinity();
		(*this)[UPPER][X] = -std::numeric_limits<float>::infinity();
		(*this)[UPPER][Y] = -std::numeric_limits<float>::infinity();
	} // end method

	// Increases this rectangle so that the point x, y will be inside it.

	void stretchToFit(const float x, const float y) {
		(*this)[LOWER] = min((*this)[LOWER], float2(x, y));
		(*this)[UPPER] = max((*this)[UPPER], float2(x, y));
	} // end method

	void stretchToFit(const float2 p) {
		stretchToFit(p.x, p.y);
	} // end method

	void clear () {
		clsPoints[0].clear();
		clsPoints[1].clear();
	} // end method

}; // end struct


#endif

