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
 
#include <Rsyn/PhysicalDesign>
#include "PhysicalRouting.h"

namespace Rsyn {

// -----------------------------------------------------------------------------
// PhysicalRoutingWire
// -----------------------------------------------------------------------------

PhysicalRoutingWire::PhysicalRoutingWire(const PhysicalRoutingWire & wire) {
	clsLayer = wire.clsLayer;
	clsPoints = wire.clsPoints;
	clsSourceExtension = wire.clsSourceExtension;
	clsTargetExtension = wire.clsTargetExtension;
	clsWidth = wire.clsWidth;
} // end constructor 

// -----------------------------------------------------------------------------

DBUxy
PhysicalRoutingWire::getExtendedSourcePosition() const {
	const int numPoints = getNumPoints();
	if (numPoints == 0) {
		return DBUxy(0, 0);
	} else if (numPoints == 1) {
		return clsPoints[0];
	} else {
		return getExtendedPosition(clsPoints[0], clsPoints[1], getSourceExtension());
	} // end else
} // end method

// -----------------------------------------------------------------------------

DBUxy
PhysicalRoutingWire::getExtendedTargetPosition() const {
	const int numPoints = getNumPoints();
	if (numPoints == 0) {
		return DBUxy(0, 0);
	} else if (numPoints == 1) {
		return clsPoints[0];
	} else {
		return getExtendedPosition(clsPoints[numPoints - 1], clsPoints[numPoints - 2], getTargetExtension());
	} // end else
} // end method

// -----------------------------------------------------------------------------

DBU
PhysicalRoutingWire::getSourceExtension() const {
	return clsSourceExtension == DefaultRoutingWireExtension?
		getLayer().getWidth() / 2 : clsSourceExtension;
} // end method

// -----------------------------------------------------------------------------

DBU
PhysicalRoutingWire::getTargetExtension() const {
	return clsTargetExtension == DefaultRoutingWireExtension?
		getLayer().getWidth() / 2 : clsTargetExtension;
} // end method

// -----------------------------------------------------------------------------

std::vector<DBUxy>
PhysicalRoutingWire::getPoints(const bool useWireExtension) const {
	std::vector<DBUxy> points = clsPoints;
	if (useWireExtension && getNumPoints() >= 1) {
		points.front() = getExtendedSourcePosition();
		points.back() = getExtendedTargetPosition();
	} // end method
	return points;
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingWire::hasNonDefaultWidth() const {
	return clsWidth != DefaultRoutingWireWidth && clsWidth != clsLayer.getWidth();
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingWire::hasNonDefaultSourceExtension() const {
	return clsSourceExtension != DefaultRoutingWireExtension;
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingWire::hasNonDefaultTargetExtension() const {
	return clsTargetExtension != DefaultRoutingWireExtension;
} // end method

// -----------------------------------------------------------------------------

Rsyn::PhysicalLayer
PhysicalRoutingWire::getLayer() const {
	return clsLayer;
} // end method

// -----------------------------------------------------------------------------

DBU 
PhysicalRoutingWire::getWidth() const {
	return clsWidth == DefaultRoutingWireWidth?
		getLayer().getWidth() : clsWidth;
} // end method

// -----------------------------------------------------------------------------

DBUxy
PhysicalRoutingWire::getExtendedPosition(
		const DBUxy p0,
		const DBUxy p1,
		const DBU extension
) const {
	DBUxy pos = p0;

	const DBUxy d = p1 - p0;
	const bool horizontal = d.x != 0;
	const bool vertical = d.y != 0;
	if (horizontal && !vertical) {
		// Horizontal
		if (d.x > 0) {
			// p0.x < p1.x
			pos.x -= extension;
		} else {
			// p0.x > p1.x
			pos.x += extension;
		} // end else
	} else if (vertical && !horizontal) {
		// Vertical
		if (d.y > 0) {
			// p0.y < p1.y
			pos.y -= extension;
		} else {
			// p0.y > p1.y
			pos.y += extension;
		} // end else
	} // end else-if

	return pos;
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingWire::isValid() const {
	if (!getLayer() || getLayer().getType() != Rsyn::ROUTING) return false;
	if (getNumPoints() < 2) return false;
	return true;
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRoutingWire::addRoutingPoint(const DBUxy p) {
	clsPoints.push_back(p);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRoutingWire::setLayer(Rsyn::PhysicalLayer layer) {
	clsLayer = layer;
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRoutingWire::clear() {
	*this = PhysicalRoutingWire();
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingWire::convertToPolygon(Rsyn::Polygon &polygon) const {
	if (isValid()) {
		polygon = Polygon::createFromLineString(getPoints(true), getWidth());
		return true;
	} else {
		polygon.clear();
		return false;
	} // end else
} // end method

// -----------------------------------------------------------------------------
// PhysicalRoutingVia
// -----------------------------------------------------------------------------

PhysicalRoutingVia::PhysicalRoutingVia(const PhysicalRoutingVia & via) {
	clsPosition = via.clsPosition;
	clsVia = via.clsVia;
} // end constructor 

// -----------------------------------------------------------------------------

Rsyn::PhysicalVia
PhysicalRoutingVia::getVia() const {
	return clsVia;
} // end method

// -----------------------------------------------------------------------------

Rsyn::PhysicalLayer
PhysicalRoutingVia::getTopLayer() const {
	return clsVia.getTopLayer();
} // end method

// -----------------------------------------------------------------------------

Rsyn::PhysicalLayer
PhysicalRoutingVia::getCutLayer() const {
	return clsVia.getCutLayer();
} // end method

// -----------------------------------------------------------------------------

Rsyn::PhysicalLayer
PhysicalRoutingVia::getBottomLayer() const {
	return clsVia.getBottomLayer();
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRoutingVia::setVia(Rsyn::PhysicalVia via) {
	clsVia = via;
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingVia::isValid() const {
	return clsVia != nullptr;
} // end method

// -----------------------------------------------------------------------------
// PhysicalRoutingRect
// -----------------------------------------------------------------------------

PhysicalRoutingRect::PhysicalRoutingRect(const PhysicalRoutingRect & rect) {
	clsLayer = rect.clsLayer;
	clsRect = rect.clsRect;
} // end constructor

// -----------------------------------------------------------------------------

Rsyn::PhysicalLayer
PhysicalRoutingRect::getLayer() const {
	return clsLayer;
} // end method

// -----------------------------------------------------------------------------

bool
PhysicalRoutingRect::isValid() const {
	return clsLayer;
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRoutingRect::setLayer(Rsyn::PhysicalLayer layer) {
	clsLayer = layer;
} // end method

// -----------------------------------------------------------------------------
// PhysicalRouting
// -----------------------------------------------------------------------------

PhysicalRouting::PhysicalRouting(const PhysicalRouting & routing) {
	clsWires = routing.clsWires;
	clsVias = routing.clsVias;
	clsRects = routing.clsRects;
} // end constructor 

// -----------------------------------------------------------------------------

void
PhysicalRouting::addWire(const PhysicalRoutingWire &wire) {
	assert(wire.isValid());
	clsWires.push_back(wire);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRouting::addWire(
		Rsyn::PhysicalLayer layer,
		const DBUxy source,
		const DBUxy target,
		const DBU width,
		const DBU sourceExtension,
		const DBU targetExtension
) {
	PhysicalRoutingWire wire;
	wire.setLayer(layer);
	wire.addRoutingPoint(source);
	wire.addRoutingPoint(target);
	wire.setWidth(width);
	wire.setSourceExtension(sourceExtension);
	wire.setTargetExtension(targetExtension);
	addWire(wire);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRouting::addWire(
		Rsyn::PhysicalLayer layer,
		const std::vector<DBUxy> &points,
		const DBU width,
		const DBU sourceExtension,
		const DBU targetExtension
) {
	PhysicalRoutingWire wire;
	wire.setLayer(layer);
	for (const DBUxy point : points)
		wire.addRoutingPoint(point);
	wire.setWidth(width);
	wire.setSourceExtension(sourceExtension);
	wire.setTargetExtension(targetExtension);
	addWire(wire);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRouting::addVia(
		const PhysicalRoutingVia &via
) {
	clsVias.push_back(via);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRouting::addVia(
		Rsyn::PhysicalVia physicalVia,
		const DBUxy position
) {
	PhysicalRoutingVia via;
	via.setVia(physicalVia);
	via.setPosition(position);
	addVia(via);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRouting::addRect(
		const PhysicalRoutingRect &rect
) {
	clsRects.push_back(rect);
} // end method

// -----------------------------------------------------------------------------

void
PhysicalRouting::addRect(
		Rsyn::PhysicalLayer layer,
		const Bounds &rect
) {
	PhysicalRoutingRect patch;
	patch.setLayer(layer);
	patch.setRect(rect);
	addRect(patch);
} // end method

// -----------------------------------------------------------------------------

bool PhysicalRouting::isValid() const {
	for (const PhysicalRoutingWire &wire : allWires()) {
		if (!wire.isValid()) {
			return false;
		} // end if
	} // end for

	for (const PhysicalRoutingVia &via : allVias()) {
		if (!via.isValid()) {
			return false;
		} // end if
	} // end for

	return !(clsWires.empty() && clsVias.empty());
} // end method

// -----------------------------------------------------------------------------

bool PhysicalRouting::isEmpty() const {
	return clsWires.empty() && clsVias.empty() && clsRects.empty();
} // end method

// -----------------------------------------------------------------------------

void 
PhysicalRouting::clear() {
	clsWires.clear();
	clsVias.clear();
	clsRects.clear();
} // end method

// -----------------------------------------------------------------------------

DBU PhysicalRouting::computeWirelength() const {
	DBU wirelength = 0;
	for (Rsyn::PhysicalRoutingWire wire : allWires()) {
		wirelength += DBUxy::computeManhattanDistance(wire.allPoints()[0], wire.allPoints()[1]);
	} // end for
	return wirelength;
} // end method

// -----------------------------------------------------------------------------

} // end namespace
