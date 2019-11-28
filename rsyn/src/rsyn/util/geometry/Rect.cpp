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

#include "Rect.h"
#include "Polygon.h"

namespace Rsyn {

// -----------------------------------------------------------------------------

bool
Rect::overlaps(const Polygon &polygon) const {
	return polygon.overlaps(*this);
} // end method

// -----------------------------------------------------------------------------

Polygon
Rect::toPolygon() const {
	Polygon polygon;
	polygon.addPoint(getLower().getX(), getLower().getY());
	polygon.addPoint(getUpper().getX(), getLower().getY());
	polygon.addPoint(getUpper().getX(), getUpper().getY());
	polygon.addPoint(getLower().getX(), getUpper().getY());
	return polygon;
} // end method

} // end namespace