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
 
#ifndef RSYN_DOUBLE2_H
#define	RSYN_DOUBLE2_H

#include <cmath>

#include <algorithm>
using std::max;
using std::min;
#include <ostream>
using std::ostream;
#include <vector>
using std::vector;

#include "dim.h"
#include "dbu.h"
#include "FloatingPoint.h"

// -----------------------------------------------------------------------------

#define MAKE_SELF_OPERATOR( OP ) \
	inline void operator OP ( const double2 right ) { x OP right.x; y OP right.y; } 

#define MAKE_SELF_OPERATOR_SCALAR( OP ) \
	inline void operator OP ( const double right ) { x OP right; y OP right; }

#define MAKE_OPERATOR( OP ) \
	inline double2 operator OP ( const double2 v0, const double2 v1 ) { \
		return double2( v0.x OP v1.x, v0.y OP v1.y ); \
	}

#define MAKE_OPERATOR_SCALAR( OP ) \
	inline double2 operator OP ( const double scalar, const double2 v ) { \
		return double2( v.x OP scalar, v.y OP scalar ); \
	} \
	inline double2 operator OP ( const double2 v, const double scalar ) { \
		return double2( v.x OP scalar, v.y OP scalar ); \
	}

#define MAKE_FUNCTION_1( FUNC ) \
	inline double2 FUNC ( const double2 v ) { \
		return double2( FUNC (v.x), FUNC (v.y) ); \
	}

#define MAKE_FUNCTION_2( FUNC ) \
	inline double2 FUNC ( const double2 v0, const double2 v1 ) { \
		return double2( FUNC (v0.x, v1.x), FUNC (v0.y, v1.y) ); \
	}
	
struct double2 {
	friend ostream &operator<<( ostream &out, const double2 &v ) {
		return out << "(" << v.x << ", " << v.y << ")";
	} // end function	
	
	union {
		struct { double x, y; };
		double xy[2];
	};

	// Construtctors.
	double2(): x(0.0), y(0.0)  {}
	double2( const double x, const double y ) : x(x), y(y) {}

	explicit double2( const DBUxy &xy ) : x(xy.x), y(xy.y) {}
	explicit double2( const double scalar ) : x(scalar), y(scalar) {} // explicit to avoid accidentally assigning to scalar

	DBUxy scaleAndConvertToDbu(const double scale, const RoundingStrategy roundingStrategy = ROUND_DOWN) const {
		return DBUxy(
				(DBU) FloatingPoint::round(x*scale, roundingStrategy),
				(DBU) FloatingPoint::round(y*scale, roundingStrategy));
	} // end method

	DBUxy convertToDbu(const RoundingStrategy roundingStrategy = ROUND_DOWN) const {
		return DBUxy(
				(DBU) FloatingPoint::round(x, roundingStrategy),
				(DBU) FloatingPoint::round(y, roundingStrategy));
	} // end method

	// Operators.
	MAKE_SELF_OPERATOR( += );
	MAKE_SELF_OPERATOR( -= );
	MAKE_SELF_OPERATOR( *= );
	MAKE_SELF_OPERATOR( /= );
	
	MAKE_SELF_OPERATOR( = );
	
	inline const bool operator == ( const double2 v ) const {
		return x == v.x && y == v.y;
	} // end method
	
	inline const bool operator != ( const double2 v ) const {
		return x != v.x || y != v.y;
	} // end method
	
	MAKE_SELF_OPERATOR_SCALAR( *= );
	MAKE_SELF_OPERATOR_SCALAR( /= );
	
	      double &operator[](const int dimension)       { return xy[dimension]; }
	const double &operator[](const int dimension) const { return xy[dimension]; }
	
	// Methods.
	double norm() const {
		return sqrt( x*x + y*y );
	} // end method
	
	double2 normalized() const {
		const double v = norm();
		return double2( x/v, y/v );
	} // end method

	double2 safeNormalized() const {
		const double v = norm();
		if ( v == 0.0 )
			return double2(0,0);
		else
			return double2( x/v, y/v );
	} // end method	
	
	void apply(const double scalar) {
		x = scalar;
		y = scalar;
	} // end method
	
	void set( const double x, const double y ) {
		this->x = x;
		this->y = y;
	} // end method
	
	void scale(const double numb){
		x = x*numb;
		y = y*numb;
	} // end method 
	
	double aggregated() const { 
		return x + y; 
	} // end method 
	
	void abs () {
		x = std::abs(x);
		y = std::abs(y);
	} // end method 
	
	void clear () {
		x = 0;
		y = 0;
	} // end method 
	
	inline bool approximatelyEqual(const double2 other, const double precision = 1e-6) const;
	
}; // end struct

MAKE_OPERATOR( + );
MAKE_OPERATOR( - );
MAKE_OPERATOR( * );
MAKE_OPERATOR( / );

MAKE_OPERATOR_SCALAR( * );
MAKE_OPERATOR_SCALAR( / );

MAKE_FUNCTION_2(max);
MAKE_FUNCTION_2(min);

MAKE_FUNCTION_1(sqrt);

// Unary operators.
inline double2 operator-( const double2 value ) {
	return double2( -value.x, -value.y );
} // end operator

inline double2 operator+( const double2 value ) {
	return double2( +value.x, +value.y );
} // end operator

// Element-wise power.
inline double2 pow( const double2 base, const double exp ) {
	return double2( pow( base.x, exp ), pow( base.y, exp ) );
} // end function

// -----------------------------------------------------------------------------

inline bool double2::approximatelyEqual(const double2 other, const double precision) const {
	return (*this - other).norm() <= precision;
} // end method

// -----------------------------------------------------------------------------

// The split vector class emulates in some extent a vector<double2>. However
// x and y values are stored in two separated arrays (vector<double>). When
// accessing elements through [] operator, the split vector class creates a 
// special reference to double2 which allows seamlessly operation with double2
// variables. But, as x and y are split, the split vector class keeps
// compatibility with frameworks which do not support double2 (e.g. linear 
// algebra ones).

// @note Deprecated...

class double2_split_vector {
public:	
	struct double2_ref {
		double &x;
		double &y;

		double2_ref( double &x, double &y ) : x(x), y(y) {}

		MAKE_SELF_OPERATOR( += );
		MAKE_SELF_OPERATOR( -= );
		MAKE_SELF_OPERATOR( *= );
		MAKE_SELF_OPERATOR( /= );

		MAKE_SELF_OPERATOR( = );

		      double &operator[](const int dimension)       { return (dimension==X)? x : ( (dimension==Y)? y : *((double *)NULL) ); }
		const double &operator[](const int dimension) const { return (dimension==X)? x : ( (dimension==Y)? y : *((double *)NULL) ); }

		// Returns a const double2 to avoid accidentally trying to assign to a
		// temporary.
		operator const double2() const { return double2(x,y); } 
	}; // end struct	

	vector<double> xy[2];
	
	const double2 operator[](const int index) const { return double2(xy[0][index],xy[1][index]); }
	double2_ref operator[](const int index) { return double2_ref(xy[0][index],xy[1][index]); }
	
	void resize(size_t size) { xy[0].resize(size); xy[1].resize(size); }
	void resize(size_t size, double2 val) { xy[0].resize(size, val.x); xy[1].resize(size, val.y); }
	
	void assign(size_t n, double2 val ) { xy[0].assign(n, val.x); xy[1].assign(n, val.y); }
	
	void clear() { xy[0].clear(); xy[1].clear(); } 
	
	size_t size() const { return xy[0].size(); }
	
}; // end class

// -----------------------------------------------------------------------------

#undef MAKE_OPERATOR
#undef MAKE_OPERATOR_SCALAR
#undef MAKE_SELF_OPERATOR
#undef MAKE_SELF_OPERATOR_SCALAR
#undef MAKE_FUNCTION_1
#undef MAKE_FUNCTION_2

#endif
