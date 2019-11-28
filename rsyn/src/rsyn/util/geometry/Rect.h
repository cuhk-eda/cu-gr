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

#ifndef RSYN_GEOMETRY_RECT_H
#define RSYN_GEOMETRY_RECT_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "rsyn/util/dbu.h"
#include "rsyn/util/Bounds.h"
#include "Point.h"

namespace Rsyn {

class Polygon;

class Rect {
public:

	Rect() : clsLower(0, 0), clsUpper(0, 0) {}
	Rect(const DBU x, const DBU y, const DBU w, const DBU h) : clsLower(x, y), clsUpper(x + w, y + h) {}
	Rect(const Bounds &bounds) : clsLower(bounds.getLower()), clsUpper(bounds.getUpper()) {}

	DBU getX() const {return clsLower.getX();}
	DBU getY() const {return clsLower.getY();}
	DBU getWidth() const {return clsUpper.getX() - clsLower.getX();}
	DBU getHeight() const {return clsUpper.getY() - clsLower.getY();}

	Point getLower() const {return clsLower;}
	Point getUpper() const {return clsUpper;}

	DBU getArea() const {return getWidth() * getHeight();}
	
	void setX(const DBU x) {
		const DBU w = getWidth();
		clsLower.setX(x);
		clsUpper.setX(x + w);
	} // end method

	void setY(const DBU y) {
		const DBU h = getHeight();
		clsLower.setY(y);
		clsUpper.setY(y + h);
	} // end method

	void setWidth(const DBU w) {
		clsUpper.setX(clsLower.getX() + w);
	} // end method

	void setHeight(const DBU h) {
		clsUpper.setY(clsLower.getY() + h);
	} // end method

	//! @brief Translates this rectangle by (dx, dy).
	void translate(const DBU dx, const DBU dy) {
		clsLower.translate(dx, dy);
		clsUpper.translate(dx, dy);
	} // end method

	//! @brief Translates this rectangle by displacement.
	void translate(const DBUxy displacement) {
		translate(displacement.x, displacement.y);
	} // end method

	//! @brief Returns copy of this rectangle translated by (dx, dy).
	Rect translated(const DBU dx, const DBU dy) const {
		Rect rect = *this;
		rect.translate(dx, dy);
		return rect;
	} // end method

	//! @brief Returns copy of this rectangle translated by displacement.
	Rect translated(const DBUxy displacement) const {
		return translated(displacement.x, displacement.y);
	} // end method

	bool contains(const DBU x, const DBU y) const {
		return (x >= clsLower.getX() && x <= clsUpper.getX()) &&
				(y >= clsLower.getY() && y <= clsUpper.getY());
	} // end method

	bool contains(const Point &point) const {
		return contains(point.getX(), point.getY());
	} // end method

	bool overlaps(const Rect &rect) const {
		const DBU dx = std::max(getLower().getX(), rect.getLower().getX()) -
				std::min(getUpper().getX(), rect.getUpper().getX());
		const DBU dy = std::max(getLower().getY(), rect.getLower().getY()) -
				std::min(getUpper().getY(), rect.getUpper().getY());
		return (dx<=0 && dy<=0);
	} // end method

	bool overlaps(const Polygon &polygon) const;

	operator Bounds() const {return Bounds(getLower(), getUpper());}
	
	Bounds getBounds() const { return Bounds(getLower(), getUpper()); }

	Polygon toPolygon() const;

private:

	Point clsUpper;
	Point clsLower;

}; // end class

} // end namespace

BOOST_GEOMETRY_REGISTER_BOX(Rsyn::Rect, Rsyn::Point, getLower(), getUpper());

#endif

