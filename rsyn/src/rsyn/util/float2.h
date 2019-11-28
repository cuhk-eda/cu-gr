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
 
#ifndef RSYN_FLOAT2_H
#define	RSYN_FLOAT2_H

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
	inline void operator OP ( const float2 right ) { x OP right.x; y OP right.y; }

#define MAKE_SELF_OPERATOR_SCALAR( OP ) \
	inline void operator OP ( const float right ) { x OP right; y OP right; }

#define MAKE_OPERATOR( OP ) \
	inline float2 operator OP ( const float2 v0, const float2 v1 ) { \
		return float2( v0.x OP v1.x, v0.y OP v1.y ); \
	}

#define MAKE_OPERATOR_SCALAR( OP ) \
	inline float2 operator OP ( const float scalar, const float2 v ) { \
		return float2( v.x OP scalar, v.y OP scalar ); \
	} \
	inline float2 operator OP ( const float2 v, const float scalar ) { \
		return float2( v.x OP scalar, v.y OP scalar ); \
	}

#define MAKE_FUNCTION_1( FUNC ) \
	inline float2 FUNC ( const float2 v ) { \
		return float2( std::FUNC (v.x), std::FUNC (v.y) ); \
	}

#define MAKE_FUNCTION_2( FUNC ) \
	inline float2 FUNC ( const float2 v0, const float2 v1 ) { \
		return float2( std::FUNC (v0.x, v1.x), std::FUNC (v0.y, v1.y) ); \
	}
	
struct float2 {
	friend ostream &operator<<( ostream &out, const float2 &v ) {
		return out << "(" << v.x << ", " << v.y << ")";
	} // end function	
	
	union {
		struct { float x, y; };
		float xy[2];
	};

	// Construtctors.
	float2(): x(0), y(0)  {}
	float2( const float x, const float y ) : x(x), y(y) {}

	explicit float2( const DBUxy &xy ) : x(xy.x), y(xy.y) {}
	explicit float2( const float scalar ) : x(scalar), y(scalar) {} // explicit to avoid accidentally assigning to scalar

	DBUxy scaleAndConvertToDbu(const float scale, const RoundingStrategy roundingStrategy = ROUND_DOWN) const {
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
	
	inline const bool operator == ( const float2 v ) const {
		return x == v.x && y == v.y;
	} // end method
	
	inline const bool operator != ( const float2 v ) const {
		return x != v.x || y != v.y;
	} // end method
	
	MAKE_SELF_OPERATOR_SCALAR( *= );
	MAKE_SELF_OPERATOR_SCALAR( /= );
	
	      float &operator[](const int dimension)       { return xy[dimension]; }
	const float &operator[](const int dimension) const { return xy[dimension]; }
	
	// Methods.
	float norm() const {
		return std::sqrt( x*x + y*y );
	} // end method
	
	float2 normalized() const {
		const float v = norm();
		return float2( x/v, y/v );
	} // end method

	float2 safeNormalized() const {
		const float v = norm();
		if ( v == 0.0 )
			return float2(0,0);
		else
			return float2( x/v, y/v );
	} // end method

	// clockwise
	float2 perpendicular() const {
		return float2( y, -x );
	} // end method

	void apply(const float scalar) {
		x = scalar;
		y = scalar;
	} // end method
	
	void set( const float x, const float y ) {
		this->x = x;
		this->y = y;
	} // end method
	
	void scale(const float numb){
		x = x*numb;
		y = y*numb;
	} // end method 
	
	float aggregated() const {
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
	
	inline bool approximatelyEqual(const float2 other, const float precision = 1e-6f) const;
	
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
inline float2 operator-( const float2 value ) {
	return float2( -value.x, -value.y );
} // end operator

inline float2 operator+( const float2 value ) {
	return float2( +value.x, +value.y );
} // end operator

// Element-wise power.
inline float2 pow( const float2 base, const float exp ) {
	return float2( std::pow( base.x, exp ), std::pow( base.y, exp ) );
} // end function

// -----------------------------------------------------------------------------

inline bool float2::approximatelyEqual(const float2 other, const float precision) const {
	return (*this - other).norm() <= precision;
} // end method

// -----------------------------------------------------------------------------

#undef MAKE_OPERATOR
#undef MAKE_OPERATOR_SCALAR
#undef MAKE_SELF_OPERATOR
#undef MAKE_SELF_OPERATOR_SCALAR
#undef MAKE_FUNCTION_1
#undef MAKE_FUNCTION_2

#endif
