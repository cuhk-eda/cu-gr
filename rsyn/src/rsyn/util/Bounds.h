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
 
#ifndef RSYN_BOUNDS_H
#define RSYN_BOUNDS_H

#include <algorithm>
#include <limits>

#include "dim.h"
#include "dbu.h"

// -----------------------------------------------------------------------------

class Bounds {
	friend ostream &operator<<( std::ostream &out, const Bounds &r ) {
		return out << "[" << r[LOWER] << ", " << r[UPPER] << "]";
	} // end function		
	
private:
	DBUxy clsPoints[2];
public:
	
	Bounds() {
		clsPoints[0] = DBUxy();
		clsPoints[1] = DBUxy();
	}
	
	Bounds(const DBU xmin, const DBU ymin, const DBU xmax, const DBU ymax ) {
		(*this)[LOWER].set(xmin,ymin);
		(*this)[UPPER].set(xmax,ymax);
	} // end constructor	
	
	Bounds(const DBUxy pmin, const DBUxy pmax ) {
		(*this)[LOWER] = pmin;
		(*this)[UPPER] = pmax;
	} // end constructor	
	
	void updatePoints(const DBUxy pmin, const DBUxy pmax){
		(*this)[LOWER] = pmin;
		(*this)[UPPER] = pmax;
	}
	void updatePoints(const DBU xMin, const DBU yMin, const DBU xMax, const DBU yMax){
		(*this)[LOWER][X] = xMin;
		(*this)[LOWER][Y] = yMin;
		(*this)[UPPER][X] = xMax;
		(*this)[UPPER][Y] = yMax;
	}
	DBUxy &operator[](const int boundary) { return clsPoints[boundary]; }
	const DBUxy &operator[](const int boundary) const { return clsPoints[boundary]; }
	
	DBU computeLength(const int dimension) const { return (*this)[UPPER][dimension] - (*this)[LOWER][dimension]; }
	DBUxy computeLength() const { return (*this)[UPPER] - (*this)[LOWER]; }
	
	FloatingPointDBU computeDiagonal() const { return std::sqrt( std::pow(computeLength(X), 2) + std::pow(computeLength(Y), 2) ); }
	
	DBU computeCenter(const int dimension) const { return ( (*this)[UPPER][dimension] + (*this)[LOWER][dimension] ) / 2; }
	DBUxy computeCenter() const { return ( (*this)[UPPER] + (*this)[LOWER] ) / 2; }
	
	DBU computeArea() const { return computeLength(0)*computeLength(1); }
	DBU computeSemiperimeter() const { return computeLength(0)+computeLength(1); }
	
	DBU randomInnerPoint(const int dimension) const {
		const FloatingPointDBU random = rand()/double(RAND_MAX);
		return (DBU) ((*this)[LOWER][dimension] + random*computeLength(dimension));
	} // end method

	DBU overlapArea( const Bounds &rect ) const {
		const DBU dx = std::max( (*this)[LOWER][X], rect[LOWER][X] ) - std::min( (*this)[UPPER][X], rect[UPPER][X] );
		const DBU dy = std::max( (*this)[LOWER][Y], rect[LOWER][Y] ) - std::min( (*this)[UPPER][Y], rect[UPPER][Y] );

		return (dx<0 && dy<0) ? dx*dy : 0;
	} // end method
	
	DBU getCoordinate(const Boundary bound, const Dimension dim ) const { return (*this)[bound][dim]; }
	DBUxy getCoordinate(const Boundary bound ) const { return (*this)[bound]; }

	DBU getX() const { return (*this)[LOWER][X]; }
	DBU getY() const { return (*this)[LOWER][Y]; }
	DBU getWidth() const { return computeLength(X); }
	DBU getHeight() const { return computeLength(Y); }

	DBUxy getLower() const { return (*this)[LOWER]; }
	DBUxy getUpper() const { return (*this)[UPPER]; }

	bool overlap(const Bounds &rect) const {
		const DBU dx = std::max( (*this)[LOWER][X], rect[LOWER][X] ) - std::min( (*this)[UPPER][X], rect[UPPER][X] );
		const DBU dy = std::max( (*this)[LOWER][Y], rect[LOWER][Y] ) - std::min( (*this)[UPPER][Y], rect[UPPER][Y] );

		return (dx<0 && dy<0);
	} // end method	

	// Returns the rectangle formed by the intersection of this and other 
	// rectangle. If they don't overlap returns a degenerated rectangle (zero
	// area) at half way between the two rectangles.
	Bounds overlapRectangle(const Bounds &rect) const {
		const DBU dx = std::max( (*this)[LOWER][X], rect[LOWER][X] ) - std::min( (*this)[UPPER][X], rect[UPPER][X] );
		const DBU dy = std::max( (*this)[LOWER][Y], rect[LOWER][Y] ) - std::min( (*this)[UPPER][Y], rect[UPPER][Y] );

		Bounds overlap;
		if ( dx<0 && dy<0 ) {
			overlap[LOWER] = max( (*this)[LOWER], rect[LOWER] );
			overlap[UPPER] = min( (*this)[UPPER], rect[UPPER] );
		} else {
			// Returns a zero-area rectangle at half the way of the two
			// rectangles.
			const DBUxy upper = max( (*this)[LOWER], rect[LOWER] );
			const DBUxy lower = min( (*this)[UPPER], rect[UPPER] );
			
			overlap[LOWER] = (upper+lower) / 2;
			overlap[UPPER] = overlap[LOWER];
		} // end else

		return overlap;
	} // end method
	
	// Returns the dimension length overlap of this and other rectangle. Otherwise, returns zero 
	DBU overlapDimensionLength(const Bounds &rect, const Dimension dim ) const {
		const DBU delta = std::max( (*this)[LOWER][dim], rect[LOWER][dim] ) - std::min( (*this)[UPPER][dim], rect[UPPER][dim] );
		return delta < 0 ? -delta : 0;
	} // end method
	

	// Check if a value is in between the lower and upper bounds of this 
	// rectangle.
	bool between( const DBU pos, const Dimension DIMENSION ) const {
		return ( pos >= (*this)[LOWER][DIMENSION] && pos <= (*this)[UPPER][DIMENSION] );
	} // end method

	// Check if a point is inside this rectangle.
	
	bool inside( const DBU x, const DBU y ) const {
		return 
			( x >= (*this)[LOWER][X] && x <= (*this)[UPPER][X] ) && 
			( y >= (*this)[LOWER][Y] && y <= (*this)[UPPER][Y] );
	} // end method	
	
	bool inside( const DBUxy pos ) const {
		return between(pos[X],X) && between(pos[Y],Y);
	} // end method
	
	bool inside( const Bounds & bounds ) const {
		return inside(bounds[LOWER]) && inside(bounds[UPPER]);
	} // end method
	// Change the position of this rectangle.
	
	void moveTo( const DBU position, const Dimension DIMENSION ) {
		const DBU length = computeLength(DIMENSION);
		(*this)[LOWER][DIMENSION] = position;
		(*this)[UPPER][DIMENSION] = position + length;
	} // end method	
	
	void moveTo( const DBUxy position ) {
		const DBUxy length = computeLength();
		(*this)[LOWER] = position;
		(*this)[UPPER] = position + length; 
	} // end method
	
	void moveCenterTo( const DBUxy position ) {
		const DBUxy halfLength = computeLength() / 2;
		(*this)[LOWER] = position - halfLength;
		(*this)[UPPER] = position + halfLength; 
	} // end method	
	
	void translate( const DBUxy displacement ) {
		(*this)[LOWER] += displacement;
		(*this)[UPPER] += displacement;
	} // end method
	
	void moveTo( const DBU x, const DBU y ) { moveTo(DBUxy(x,y)); }
	void moveCenterTo( const DBU x, const DBU y ) { moveCenterTo(DBUxy(x,y)); }
	void translate( const DBU x, const DBU y ) { translate(DBUxy(x,y)); }

	Bounds getTranslated(const DBUxy displacement) const {
		Bounds bounds = *this;
		bounds.translate(displacement);
		return bounds;
	} // end method

	Bounds getTranslated(const DBU x, const DBU y) const {
		return getTranslated(DBUxy(x, y));
	} // end method

	// Multiple each coordinates by a scale factor. Useful when change the
	// coordinate system.
	void scaleCoordinates(const FloatingPointDBU scaling) {
		clsPoints[LOWER].scale(scaling);
		clsPoints[UPPER].scale(scaling);
	} // end method 
	
	// Scale this rectangle using its center as the point of reference. Note
	// that this will keep the center of the rectangle in the same position.
	
	void scaleCentralized(const FloatingPointDBU xFactor, const FloatingPointDBU yFactor) {
		const DBUxy p = computeCenter();
		translate(-p);
		clsPoints[LOWER].scale(xFactor, yFactor);
		clsPoints[UPPER].scale(xFactor, yFactor);
		translate(+p);
	} // end method
	
	void scaleCentralized(const FloatingPointDBU factor) {
		scaleCentralized(factor, factor);
	} // end method

	// Change size.
	void setLength(const Dimension DIMENSION, const DBU length) {
		clsPoints[UPPER][DIMENSION] = clsPoints[LOWER][DIMENSION] + length;
	} // end method

	// Check if this rectangle is valid, that is the lower coordinates are
	// smaller than the upper coordinates.
	bool isValid() const {
		return 
			(*this)[LOWER][X] <= (*this)[UPPER][X] &&
			(*this)[LOWER][Y] <= (*this)[UPPER][Y];
	} // end method
	
	// Get the point inside the rectangle which is closest to p.
	DBUxy closestPoint(const DBUxy p) const {
		DBUxy lower = getCoordinate(LOWER);
		DBUxy upper = getCoordinate(UPPER);
		return max(min(p, upper), lower);
	} // end method

	// Make the lower be +inf and upper be -inf. Useful when computing the
	// bounding box of a set of points.
	void degenerate() {
		(*this)[LOWER][X] = std::numeric_limits<DBU>::max();
		(*this)[LOWER][Y] = std::numeric_limits<DBU>::max();
		(*this)[UPPER][X] = std::numeric_limits<DBU>::min();
		(*this)[UPPER][Y] = std::numeric_limits<DBU>::min();
	} // end method
	
	// Increases this rectangle so that the point x, y will be inside it.
	
	void stretchToFit(const DBU x, const DBU y) {
		(*this)[LOWER] = min((*this)[LOWER], DBUxy(x, y));
		(*this)[UPPER] = max((*this)[UPPER], DBUxy(x, y));
	} // end method
	
	void stretchToFit(const DBUxy p) {
		stretchToFit(p.x, p.y);
	} // end method
	
	void clear () {
		clsPoints[0].clear();
		clsPoints[1].clear();
	} // end method 
	
}; // end struct


#endif

