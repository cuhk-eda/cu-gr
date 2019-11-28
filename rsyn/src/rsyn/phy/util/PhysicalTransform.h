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
 
#ifndef RSYN_PHYSICAL_TRANSFORMATION_H
#define RSYN_PHYSICAL_TRANSFORMATION_H

#include "rsyn/core/RsynTypes.h"
#include "rsyn/util/dbu.h"
#include "rsyn/util/Bounds.h"

#include <Rsyn/Polygon>

namespace Rsyn {

class PhysicalTransform {
public:

	PhysicalTransform() : clsOrientation(ORIENTATION_INVALID) {}

	//! @brief Creates a transform with origin at (0, 0).
	//! Example of Use: Transform port geometries.
	PhysicalTransform(const PhysicalOrientation orientation) : clsBounds(0, 0, 0, 0), clsOrientation(orientation) {}
	
	//! @brief Creates a transform with origin at (origin.x, origin.y).
	PhysicalTransform(const DBUxy origin, const PhysicalOrientation orientation) : clsBounds(origin.x, origin.y, 0, 0), clsOrientation(orientation) {}

	//! @briief Creates a transform with origin at the lower-left point of the
	//! bounds with the necessary translation, obtained from the bounds size, to
	//! keep the lower-left position of the transformed bounds at the same
	//! position of the lower-left of the original bounds.
	//! Example of Use: Handle cell transformation where one expects that the
	//! cell position (lower-left) to be kept at the same position after the
	//! transformation. For instance, if one is flipping the boundary of a cell
	//! at (0, 0), the flipped cell boundary should still be at (0, 0) after the
	//! transformation.
	PhysicalTransform(const Bounds bounds, const PhysicalOrientation orientation) : clsBounds(bounds), clsOrientation(orientation) {}

	DBUxy apply(const DBUxy &point) const {
		const DBU &x = point.x - clsBounds.getX();
		const DBU &y = point.y - clsBounds.getY();
		const DBU &w = clsBounds.getWidth();
		const DBU &h = clsBounds.getHeight();
		const PhysicalOrientation &o =  clsOrientation == ORIENTATION_INVALID?
			ORIENTATION_N : clsOrientation;

		// Compute transformation.
		const DBU (&M)[2][2] = TRANSFORMATION_MATRIXES[o];
		const DBU mx = (x*M[0][0]) + (y*M[0][1]);
		const DBU my = (x*M[1][0]) + (y*M[1][1]);

		// Compute translation.
		const DBU (&T)[2][2] = TRANSLATION_MATRIXES[o];
		const DBU tx = (w*T[0][0]) + (h*T[0][1]);
		const DBU ty = (w*T[1][0]) + (h*T[1][1]);

		// Compute transformed point.
		return DBUxy(mx, my) + DBUxy(tx, ty) + clsBounds.getLower();
	} // end method

	DBUxy apply(const DBU &x, const DBU &y) const {
		return apply(DBUxy(x, y));
	} // end method

	Bounds apply(const Bounds &bounds) const {
		const DBUxy p0 = apply(bounds.getLower());
		const DBUxy p1 = apply(bounds.getUpper());
		return Bounds(min(p0, p1), max(p0, p1));
	} // end method

	Polygon apply(const Polygon &polygon) const {
		Polygon transformedPolygon;
		for (const Rsyn::Point &point : polygon.allPoints()) {
			transformedPolygon.addPoint(apply(point));
		} // end for
		return transformedPolygon;
	} // end method

	Bounds getBounds() const {return clsBounds;}
	PhysicalOrientation getOrientation() const {return clsOrientation;}

	void setBounds(const Bounds &bounds) {clsBounds = bounds;}
	void setOrientation(const PhysicalOrientation &orientation) {clsOrientation = orientation;}

private:

	Bounds clsBounds;
	PhysicalOrientation clsOrientation;

	static const DBU TRANSFORMATION_MATRIXES[NUM_PHY_ORIENTATION][2][2];
	static const DBU TRANSLATION_MATRIXES[NUM_PHY_ORIENTATION][2][2];
}; // end class

} // end namespace

#endif /* TRANSFORMATION_H */

