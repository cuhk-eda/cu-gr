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
 
#ifndef RSYN_PHYSICAL_ROUTING_H
#define RSYN_PHYSICAL_ROUTING_H

#include <vector>

#include "rsyn/util/dbu.h"
#include "rsyn/util/Bounds.h"

namespace Rsyn {

class PhysicalLayer;
class PhysicalVia;

const DBU DefaultRoutingWireExtension = -1;
const DBU DefaultRoutingWireWidth = -1;

// -----------------------------------------------------------------------------
// PhysicalRoutingWire
// -----------------------------------------------------------------------------

class PhysicalRoutingWire {
public:
	PhysicalRoutingWire() {};
	PhysicalRoutingWire(const PhysicalRoutingWire & wire);
	int getNumPoints() const {return (int) clsPoints.size();}
	DBUxy getExtendedSourcePosition() const;
	DBUxy getExtendedTargetPosition() const;

	DBU getWidth() const;
	DBU getSourceExtension() const;
	DBU getTargetExtension() const;

	std::vector<DBUxy> getPoints(const bool useWireExtension) const;

	bool hasNonDefaultWidth() const;
	bool hasNonDefaultSourceExtension() const;
	bool hasNonDefaultTargetExtension() const;

	Rsyn::PhysicalLayer getLayer() const;

	bool isValid() const;

	void addRoutingPoint(const DBUxy p);

	void setLayer(Rsyn::PhysicalLayer layer);
	void setWidth(const DBU width) {clsWidth = width;}
	void setSourceExtension(const DBU extension) {clsSourceExtension = extension;}
	void setTargetExtension(const DBU extension) {clsTargetExtension = extension;}

	bool convertToPolygon(Rsyn::Polygon &polygon) const;

	void clear();

	const std::vector<DBUxy> &allPoints() const {return clsPoints;}

private:

	DBUxy getExtendedPosition(const DBUxy p0, const DBUxy p1, const DBU extension) const;

	Rsyn::PhysicalLayer clsLayer;
	std::vector<DBUxy> clsPoints;
	DBU clsSourceExtension = DefaultRoutingWireExtension;
	DBU clsTargetExtension = DefaultRoutingWireExtension;
	DBU clsWidth = DefaultRoutingWireWidth;
}; // end class

// -----------------------------------------------------------------------------
// PhysicalRoutingVia
// -----------------------------------------------------------------------------

class PhysicalRoutingVia {
public:
	PhysicalRoutingVia(){};
	PhysicalRoutingVia(const PhysicalRoutingVia & via);
	
	Rsyn::PhysicalVia getVia() const;

	Rsyn::PhysicalLayer getTopLayer() const;
	Rsyn::PhysicalLayer getCutLayer() const;
	Rsyn::PhysicalLayer getBottomLayer() const;

	DBUxy getPosition() const {return clsPosition;}

	void setPosition(const DBUxy p) {clsPosition = p;}
	void setVia(Rsyn::PhysicalVia via);

	bool isValid() const;

private:

	DBUxy clsPosition;
	Rsyn::PhysicalVia clsVia;
}; // end class

// -----------------------------------------------------------------------------
// PhysicalRoutingRect
// -----------------------------------------------------------------------------

class PhysicalRoutingRect {
public:
	PhysicalRoutingRect() {};
	PhysicalRoutingRect(const PhysicalRoutingRect & rect);

	DBU getX() const {return clsRect.getX();}
	DBU getY() const { return clsRect.getY(); }
	DBU getWidth() const { return clsRect.getWidth(); }
	DBU getHeight() const { return clsRect.getHeight(); }

	const Bounds &getRect() const {return clsRect;}
	Rsyn::PhysicalLayer getLayer() const;

	bool isValid() const;

	void setRect(const Bounds &rect) {clsRect = rect;}
	void setLayer(Rsyn::PhysicalLayer layer);
	
private:

	Bounds clsRect;
	Rsyn::PhysicalLayer clsLayer;
}; // end class

// -----------------------------------------------------------------------------
// PhysicalRouting
// -----------------------------------------------------------------------------

class PhysicalRouting {
public:
	
	PhysicalRouting() {}
	PhysicalRouting(const PhysicalRouting & routing);
	
	//! @brief Adds a wire.
	void
	addWire(
			const PhysicalRoutingWire &wire);

	//! @brief Adds a wire.
	void 
	addWire(
			Rsyn::PhysicalLayer layer,
			const DBUxy source,
			const DBUxy target,
			const DBU width = DefaultRoutingWireWidth,
			const DBU sourceExtension = DefaultRoutingWireExtension,
			const DBU targetExtension = DefaultRoutingWireExtension);

	//! @brief Adds a wire.
	void
	addWire(
			Rsyn::PhysicalLayer layer,
			const std::vector<DBUxy> &points,
			const DBU width = DefaultRoutingWireWidth,
			const DBU sourceExtension = DefaultRoutingWireExtension,
			const DBU targetExtension = DefaultRoutingWireExtension);

	//! @brief Adds a via.
	void
	addVia(
			const PhysicalRoutingVia &via);

	//! @brief Adds a via.
	void
	addVia(
			Rsyn::PhysicalVia via,
			const DBUxy position);

	//! @brief Adds a patch rectangle.
	void
	addRect(
			const PhysicalRoutingRect &rect);

	//! @brief Adds a patch rectangle.
	void
	addRect(
			Rsyn::PhysicalLayer layer,
			const Bounds &rect);

	//! @brief Checks if this physical routing is valid by visiting all routing
	//! elements (e.g. wires and vias) and checking if they are valid.
	bool isValid() const;

	//! @brief Checks if this physical routing is empty (i.e. no wires, vias, 
	//! etc.)
	bool isEmpty() const;

	//! @brief Iterates over all wires.
	const std::vector<PhysicalRoutingWire> &
	allWires() const {return clsWires;}

	//! @brief Iterates over all vias.
	const std::vector<PhysicalRoutingVia> &
	allVias() const {return clsVias;}

	//! @brief Iterates over all patch rectangles.
	const std::vector<PhysicalRoutingRect> &
	allRects() const {return clsRects;}

	//! @brief Removes all the wires, vias and rects.
	void clear();
        
        //! @brief Computes net routed wirelength
        DBU computeWirelength() const;
	
private:

	std::vector<PhysicalRoutingWire> clsWires;
	std::vector<PhysicalRoutingVia> clsVias;
	std::vector<PhysicalRoutingRect> clsRects;

}; // end class

} // end namespace


#endif /* PHYSICALROUTING_H */

