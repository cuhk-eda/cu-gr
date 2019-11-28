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

#ifndef RSYN_GEOMETRY_POLYGON_H
#define RSYN_GEOMETRY_POLYGON_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include "rsyn/util/dbu.h"

#include <Rsyn/Point>
#include <Rsyn/Rect>

namespace Rsyn {

class Polygon {
public:

	Polygon() : clsDirty(true) {}

	//! @brief Creates a polygon from a line string.
	static Polygon createFromLineString(const std::vector<DBUxy> &points, const DBU width);

	//! @brief Returns true if this a non-empty polygon.
	bool isValid() const {
		return getNumPoints() > 0;
	} // end method

	//! @brief Adds a point to this polygon.
	void addPoint(const DBU x, const DBU y) {
		addPoint(Point(x, y));
	} // end method

	//! @brief Adds a point to this polygon.
	void addPoint(const Point &point) {
		boost::geometry::append(clsBoostPolygon.outer(), point);
		clsDirty = true;
	} // end method

	//! @brief Translates this polygon by (dx, dy).
	void translate(const DBU dx, const DBU dy);

	//! @brief Translates this polygon by displacement.
	void translate(const DBUxy displacement) {
		translate(displacement.x, displacement.y);
	} // end method

	//! @brief Returns copy of this polygon translated by (dx, dy).
	Polygon translated(const DBU dx, const DBU dy) const;

	//! @brief Returns copy of this polygon translated by displacement.
	Polygon translated(const DBUxy displacement) const {
		return translated(displacement.x, displacement.y);
	} // end method

	//! @brief Returns true if (x, y) is inside this polygon.
	bool contains(const DBU x, const DBU y) const {
		return contains(Point(x, y));
	} // end method

	//! @brief Returns true if point is inside this polygon.
	bool contains(const Point &point) const {
		correct();
		return boost::geometry::within(point, clsBoostPolygon);
	} // end method

	//! @brief Returns the shortest distance from (x, y) to this polygon.
	FloatingPointDBU distance(const DBU x, const DBU y) const {
		correct();
		return distance(Point(x, y));
	} // end method

	//! @brief Returns the shortest distance from a point to this polygon.
	FloatingPointDBU distance(const Point &point) const {
		correct();
		return boost::geometry::distance(point, clsBoostPolygon);
	} // end method

	//! @brief Returns the shortest distance from a rectangle to this polygon.
	FloatingPointDBU distance(const Rect &rect) const {
		correct();
		return boost::geometry::distance(rect, clsBoostPolygon);
	} // end method

	//! @brief Returns the shortest distance from a polygon to this polygon.
	FloatingPointDBU distance(const Polygon &poly) const {
		correct();
		return boost::geometry::distance(poly.clsBoostPolygon, clsBoostPolygon);
	} // end method

	//! @brief Returns true if a polygon to overlaps this polygon.
	bool overlaps(const Rect &rect) const {
		correct();
		// Boost 1.58 does not support overlap among a box and a polygon
		// directly so we first convert the rectangle to an equivalent polygon.
		return boost::geometry::overlaps(rect.toPolygon().clsBoostPolygon, clsBoostPolygon);
	} // end method

	//! @brief Returns true if a polygon to overlaps this polygon.
	bool overlaps(const Polygon &poly) const {
		correct();
		return boost::geometry::overlaps(poly.clsBoostPolygon, clsBoostPolygon);
	} // end method

	//! @brief Removes all points from this polygon.
	void clear() {
		clsBoostPolygon.clear();
		clsDirty = true;
	} // end method

	//! @brief Returns the number of points in this polygon.
	int getNumPoints() const {
		return boost::geometry::num_points(clsBoostPolygon);
	} // end method

	//! @brief Gets the i-th point of this polygon.
	//! @note Currently the points maybe stored in a different order than
	//! inserted as the points are corrected to be in the order boost::geometry
	//! assumes they to be.
	Point getPoint(const int index) const {
		correct();
		return clsBoostPolygon.outer()[index];
	} // end method

	//! @brief Returns a vector with all points in this polygon.
	const std::vector<Point> &getPoints() const {
		correct();
		return clsBoostPolygon.outer();
	} // end method

	//! @brief Used to iterate over all points using for-ranged loops.
	const std::vector<Point> &allPoints() const {
		correct();
		return clsBoostPolygon.outer();
	} // end method

private:

	// @todo change to boost::geometry::model::ring
	typedef boost::geometry::model::polygon<Rsyn::Point,
			false /*clock-wise*/, false /*closed*/> BoostPolygon;

	//! @brief Corrects the order of points in this polygon.
	//! @note This is necessary to comply with boost::geometry, which assumes
	//! the points of polygons are given in a specific order (e.g. clockwise).
	void correct() const {
		if (clsDirty) {
			boost::geometry::correct(clsBoostPolygon);
			clsDirty = false;
		} // end if
	} // end method

	//! @brief Indicates that this polygon may need correction as boost requires
	//! the point of a polygon to be in a certain order (e.g. clockwise).
	mutable bool clsDirty : 1;

	//! @brief The underlying Boost polygon.
	mutable BoostPolygon clsBoostPolygon;
}; // end class

} // end namespace

#endif

