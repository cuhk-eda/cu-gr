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

#include "Polygon.h"

#include <cmath>

// Note: Comment the macro below if you have some problems with
// boost::geometry::strategy::buffer.

#define USE_BOOST_TO_CONVERT_LINE_STRING_TO_POLYGON

// -----------------------------------------------------------------------------

namespace Rsyn {

// -----------------------------------------------------------------------------

void 
Polygon::translate(const DBU dx, const DBU dy) {
	for (Point &point : clsBoostPolygon.outer()) {
		point.translate(dx, dy);
	} // end for
} // end method

// -----------------------------------------------------------------------------

Polygon
Polygon::translated(const DBU dx, const DBU dy) const {
	Polygon poly = *this;
	poly.translate(dx, dy);
	return poly;
} // end method

// -----------------------------------------------------------------------------

} // end namespace

// #############################################################################
#ifdef USE_BOOST_TO_CONVERT_LINE_STRING_TO_POLYGON
// #############################################################################

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

namespace Rsyn {

Polygon
Polygon::createFromLineString(const std::vector<DBUxy> &points, const DBU width) {
	Polygon rsynPolygon;

	typedef double coordinate_type;
	typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
	typedef boost::geometry::model::polygon<point> polygon;

	// Declare strategies
	const double buffer_distance = width/2.0;
	boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
	boost::geometry::strategy::buffer::join_miter join_strategy;
	boost::geometry::strategy::buffer::end_flat end_strategy;
	boost::geometry::strategy::buffer::point_square point_strategy;
	boost::geometry::strategy::buffer::side_straight side_strategy;

	// Declare output
	boost::geometry::model::multi_polygon<polygon> result;

	// Declare/fill a linestring
	boost::geometry::model::linestring<point> ls;
	for (const DBUxy &p : points)
		boost::geometry::append(ls, point(p.x, p.y));

	// Create the buffer of a linestring
	boost::geometry::buffer(ls, result,
				distance_strategy, side_strategy,
				join_strategy, end_strategy, point_strategy);

	for (const polygon &poly : result) {
		for (const point &p : poly.outer()) {
			rsynPolygon.addPoint(DBUxy(
					static_cast<DBU>(std::round(p.x())),
					static_cast<DBU>(std::round(p.y()))));
		} // end for
		
		// Defensive programming: this operation should not return more than one
		// polygon.
		break;
	} // end for

	return rsynPolygon;
} // end method

} // end namespace

#endif

// #############################################################################
#ifndef USE_BOOST_TO_CONVERT_LINE_STRING_TO_POLYGON
// #############################################################################
// Uses the old way to convert a line string to a thick polygon inherited from
// wxSightGL implemented by Marilena Maule (Lenna) and Guilherme Flach at UFRGS
// around 2007. 
//
// Use this only if you have issues using boost::geometry::strategy::buffer,
// which is a more robust implementation and hence the preferred method.
// #############################################################################

#include "rsyn/util/float2.h"

namespace Rsyn {

float2
findIntersection(
		float2 p1,
		float2 p2,
		float2 d1,
		float2 d2
) {

	const float a = p1.x;
	const float e = p1.y;
	const float b = d1.x;
	const float f = d1.y;
	const float c = p2.x;
	const float g = p2.y;
	const float d = d2.x;
	const float h = d2.y;

	float t;
	if ((b * h - d * f) == 0) t = 0;
	else t = ((c - a) * h + (e - g) * d) / (b * h - d * f);
	float2 q = p1 + (d1 * t);

	bool bug;
	if ((t < 0 || t > 1)) {
		bug = true;
	} else {
		bug = false;
	} // end else

	return q;
} // end function

// -----------------------------------------------------------------------------

bool
findPoint(
		float2 v0,
		float2 v1,
		float2 v2,
		const float thickness,
		std::vector<float2> &outlinePoints
) {

	// Direction vectors.
	float2 d1 = v1 - v0;
	float2 d2 = v2 - v1;

	// Points over the lines.
	float2 p1 = v1 + (d1.perpendicular().normalized()) * thickness;
	float2 p2 = v1 + (d2.perpendicular().normalized()) * thickness;

	float m1 = (v0.x - v1.x) == 0 ? (v0.y - v1.y) : (v0.y - v1.y) / (v0.x - v1.x);
	float m2 = (v1.x - v2.x) == 0 ? (v1.y - v2.y) : (v1.y - v2.y) / (v1.x - v2.x);

	float tg = (m1 - m2) / (1 + m1 * m2);
	float distancia = std::sqrt(std::pow((p1.x - p2.x), 2.0f) + std::pow((p1.y - p2.y), 2.0f));
	float limite = std::sqrt(2.0f * std::pow(thickness, 2.0f));

	if (distancia > limite && tg > 0.0f) { // dois
		outlinePoints.push_back(p1);
		outlinePoints.push_back(p2);
		return true;
	} else {
		float2 q = findIntersection(p1, p2, d1, d2); // um
		outlinePoints.push_back(q);
		return true;
	}//end else
} // end method

// -----------------------------------------------------------------------------

void
tracePathOutline(
		const std::vector<DBUxy> &pathPoints,
		const float halfThickness,
		std::vector<float2> &outlinePoints
) {

	// Historical Note: This function was adapted from wxSightGL developed by
	// Lenna and Guilherme around 2007.

	float2 v1;

	// First point
	v1 = float2( pathPoints[0] ) +
			( float2( pathPoints[1] ) - float2( pathPoints[0] ) ).perpendicular().normalized() * halfThickness;
	outlinePoints.push_back(v1);

	// Forward traversal.
	bool control = true;
	for( size_t i = 1; i < pathPoints.size() - 1; i++){
		control = findPoint(
				float2(pathPoints[i-1]),
				float2(pathPoints[i  ]),
				float2(pathPoints[i+1]),
				halfThickness, outlinePoints);
	} // end for

	// Control point.
	if (control) {
		v1 = float2(pathPoints.back()) + float2( float2( pathPoints.back() ) -
				float2( pathPoints[pathPoints.size()-2] ) ).perpendicular().normalized() * halfThickness ;
		outlinePoints.push_back(v1);
	} // end if

	// Last point.
	v1 = float2( pathPoints.back() ) + float2( float2( pathPoints[pathPoints.size()-2] ) -
			float2( pathPoints.back() ) ).perpendicular().normalized() * halfThickness;
	outlinePoints.push_back(v1);

	// Backward traversal.
	for( int i = pathPoints.size() - 2; i >= 1; i--){
		findPoint(
				float2(pathPoints[i+1]),
				float2(pathPoints[i  ]),
				float2(pathPoints[i-1]),
				halfThickness, outlinePoints );
	} // end if

	v1 = float2( pathPoints[0] ) + float2( float2( pathPoints[0] ) -
			float2( pathPoints[1] ) ).perpendicular().normalized() * halfThickness ;
	outlinePoints.push_back(v1);
	v1 = float2( pathPoints[0] ) + float2( float2( pathPoints[1] ) -
			float2( pathPoints[0] ) ).perpendicular().normalized() * halfThickness ;
	outlinePoints.push_back(v1);
} // end method

// -----------------------------------------------------------------------------

Polygon
Polygon::createFromLineString(const std::vector<DBUxy> &points, const DBU width) {
	std::vector<float2> outline;
	tracePathOutline(points, width/2.0, outline);

	Polygon rsynPolygon;
	for (const float2 &p : outline) {
		rsynPolygon.addPoint(DBUxy(
				static_cast<DBU>(std::round(p.x)),
				static_cast<DBU>(std::round(p.y))));
	} // end for

	return rsynPolygon;
} // end method

} // end namespace

#endif
