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

#include <limits>

namespace Rsyn {

inline
DesignData *
Instance::_getDesignData() const {
	return getDesign().data;
} // end method

// -----------------------------------------------------------------------------

inline
DesignData *
Instance::getDesignData() {
	return _getDesignData();
} // end method

// -----------------------------------------------------------------------------

inline
const DesignData *
Instance::getDesignData() const {
	return _getDesignData();
} // end method

// -----------------------------------------------------------------------------

inline
Design
Instance::_getDesign() const {
	return data->design;
} // end method

// -----------------------------------------------------------------------------

inline
Design
Instance::getDesign() {
	return _getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
const Design
Instance::getDesign() const {
	return _getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
Module
Instance::_getParent() const {
	return data->parent;
} // end method

// -----------------------------------------------------------------------------

inline
Module
Instance::getParent() {
	return _getParent();
} // end method

// -----------------------------------------------------------------------------

inline
const Module
Instance::getParent() const {
	return _getParent();
} // end method

// -----------------------------------------------------------------------------

inline
Cell
Instance::asCell() const {
#ifdef RSYN_SAFE_MODE
	if (getType() != Rsyn::CELL)
		throw SafeModeException("Invalid instance casting. Instance is not a cell.");
#endif
	return Cell(data);
} // end method

// -----------------------------------------------------------------------------

inline
Port
Instance::asPort() const {
#ifdef RSYN_SAFE_MODE
	if (getType() != Rsyn::PORT)
		throw SafeModeException("Invalid instance casting. Instance is not a port.");
#endif
	return Port(data);
} // end method

// -----------------------------------------------------------------------------

inline
Module
Instance::asModule() const {
#ifdef RSYN_SAFE_MODE
	if (getType() != Rsyn::MODULE)
		throw SafeModeException("Invalid instance casting. Instance is not a module.");
#endif
	return Module(data);
} // end method

// -----------------------------------------------------------------------------

inline
std::string
Instance::getHierarchicalName() const {
	return (data->parent ? data->parent.getHierarchicalName() : "") + "/" + getName();
} // end method

// -----------------------------------------------------------------------------

inline
InstanceType
Instance::getType() const {
	return data->type;
} // end method

// -----------------------------------------------------------------------------

inline
Pin
Instance::getPinByName(const std::string &name) const {
	for (Rsyn::Pin pin : allPins()) {
		if (pin.getName() == name)
			return pin;
	} // end method
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
Instance::getName() const {
	return data ? getDesign()->instanceNames[data->id] : NullName;
} // end method

// -----------------------------------------------------------------------------

inline
int
Instance::getNumPins() const {
	return (int) data->pins.size();
} // end method

// -----------------------------------------------------------------------------

inline
int
Instance::getNumPins(const Direction direction) const {
	// TODO: optimize this using instance types (e.g. cell can get the number
	// of pins of certain direction via its library cell).

	int counter = 0;
	for (Rsyn::Pin pin : allPins()) {
		if (pin.getDirection() == direction) {
			counter++;
		} // end if
	} // end for

	return counter;
} // end method

// -----------------------------------------------------------------------------

inline
int
Instance::getNumInputPins() const {
	return getNumPins(Rsyn::IN);
} // end method

// -----------------------------------------------------------------------------

inline
int
Instance::getNumOutputPins() const {
	return getNumPins(Rsyn::OUT);
} // end method

// -----------------------------------------------------------------------------

inline
int
Instance::getNumBidirectionalPins() const {
	return getNumPins(Rsyn::BIDIRECTIONAL);
} // end method


// -----------------------------------------------------------------------------

inline
int
Instance::getNumArcs() const {
	return (int) data->arcs.size();
} // end method

// -----------------------------------------------------------------------------

inline
TopologicalIndex
Instance::getTopologicalIndex() const {
	// TODO: The topological order of an instance my be ambiguous in some cases
	// as in the case of registers. There's no arc between D and Q pins and 
	// hence D may have a larger topological index than Q. In a forward traverse
	// of the netlist one may expect that Q would be used as the topological
	// index of the register while in a backward traverse one would expect
	// that the D pin was used.

	// NOTE: In the current implementation the topological order of register
	// is the topological order of the clock pin.

	TopologicalIndex order = MIN_TOPOLOGICAL_INDEX;
	for (Rsyn::Arc arc : allArcs()) {
		order = std::max(order, arc.getFromPin().getTopologicalIndex());
	} // end for

	// Some cells may not have arcs, so as a fall back, get the largest
	// topological index of its pins.
	if (order == MIN_TOPOLOGICAL_INDEX) {
		for (Rsyn::Pin pin : allPins()) {
			order = std::max(order, pin.getTopologicalIndex());
		} // end for		
	} // end if

	return order;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isPort() const {
	return data->type == Rsyn::PORT;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isPort(const Direction direction) {
	return isPort() && (asPort().getDirection() == direction);
} // end method

// -----------------------------------------------------------------------------

inline
Pin
Instance::getPinByIndex(const int index) const {
	return data->pins[index];
} // end method

// -----------------------------------------------------------------------------

inline
Pin
Instance::getAnyInputPin() const {
	for (Pin pin : allPins()) {
		if (pin.getDirection() == Rsyn::IN) {
			return pin;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Pin
Instance::getAnyOutputPin() const {
	for (Pin pin : allPins()) {
		if (pin.getDirection() == Rsyn::OUT) {
			return pin;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Arc
Instance::getArc(const Pin from, const Pin to) {
	for (Arc arc : allArcs()) {
		if ((arc.getFromPin() == from) && (arc.getToPin() == to)) {
			return arc;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Arc
Instance::getArcByPinNames(const std::string &from, const std::string &to) {
	for (Arc arc : allArcs()) {
		if ((arc.getFromPin().getName() == from) && (arc.getToPin().getName() == to)) {
			return arc;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Arc
Instance::getAnyArc() {
	return !data->arcs.empty() ? data->arcs[0] : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell
Instance::getLibraryCell() const {
	return getType() == Rsyn::CELL? Cell(data).getLibraryCell() : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPinsFilteredByDirection>
Instance::allPins(const Direction direction, bool filterPG) const {
	return CollectionOfPinsFilteredByDirection(data->pins, direction, filterPG);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPins>
Instance::allPins(bool filterPG) const {
	return CollectionOfPins(data->pins, filterPG);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfArcs>
Instance::allArcs() const {
	return CollectionOfArcs(data->arcs);
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isSequential() const {
	return getType() == Rsyn::CELL ?
		asCell().getLibraryCell().isSequential() : false;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isTie(const TieType type) const {
	return getType() == Rsyn::CELL ?
		asCell().getLibraryCell().isTie(type) : false;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isBuffer(const BufferType type) const {
	return getType() == Rsyn::CELL ?
		asCell().getLibraryCell().isBuffer(type) : false;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isClockBuffer(const BufferType type, const bool local) const {
	// First check if this instance is a buffer.
	if (!isBuffer(type))
		return false;

	// Now checks if this buffer belong to a clock network.
	Rsyn::Pin pin = getAnyOutputPin();
	Rsyn::Net net = pin.getNet();

	if (!net || !net.isClockNetwork())
		return false;

	// Only consider a local clock buffer a clock buffer driving at least one
	// register.
	if (local) {
		for (Rsyn::Pin sink : net.allPins(Rsyn::SINK)) {
			Rsyn::Instance instance = sink.getInstance();
			if (instance.isSequential())
				return true;
		} // end for

		return false;
	} else {
		return true;
	} // end else
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isLCB(const BufferType type) const {
	return isClockBuffer(type, true);
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isFixed() const {
	if (data->tag.fixed.isNotSpecified())
		throw TagNotSpecifiedException("Fixed");
	return data->tag.fixed;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isMovable() const {
	return !isFixed();
} // end method

// -----------------------------------------------------------------------------

inline
bool
Instance::isMacroBlock() const {
	if (data->tag.block.isNotSpecified())
		throw TagNotSpecifiedException("Block");
	return data->tag.block;
} // end method

// -----------------------------------------------------------------------------

inline 
bool
Instance::isFlipped() const {
	PhysicalOrientation orient = getOrientation();

	const bool fliped = orient == ORIENTATION_FN || orient == ORIENTATION_FS ||
		orient == ORIENTATION_FW || orient == ORIENTATION_FE;
	return fliped;
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getX() const {
	return getPosition(X);
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getY() const {
	return getPosition(Y);
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getHeight() const {
	return data->clsBounds.computeLength(Y);
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getWidth() const {
	return data->clsBounds.computeLength(X);
} // end method

// -----------------------------------------------------------------------------

inline DBUxy Instance::getSize() const {
	return DBUxy(getWidth(), getHeight());
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getSize(const Dimension dimension) const {
	return data->clsBounds.computeLength(dimension);
} // end method

// -----------------------------------------------------------------------------

inline DBUxy Instance::getPosition() const {
	if (isPort())
		return data->clsPortPos;
	return data->clsBounds[LOWER];
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getPosition(const Dimension dim) const {
	if (isPort())
		return data->clsPortPos[dim];
	return data->clsBounds[LOWER][dim];
} // end method

// -----------------------------------------------------------------------------

inline DBUxy Instance::getCoordinate(const Boundary bound) const {
	if (isPort())
		return getPosition();
	return data->clsBounds[bound];
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getCoordinate(const Boundary bound, const Dimension dim) const {
	if (isPort())
		getPosition(dim);
	return data->clsBounds[bound][dim];
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getArea() const {
	return data->clsBounds.computeArea();
} // end method

// -----------------------------------------------------------------------------

inline DBUxy Instance::getCenter() const {
	if (isPort())
		return getPosition();
	return getBounds().computeCenter();
} // end method

// -----------------------------------------------------------------------------

inline DBU Instance::getCenter(const Dimension dim) const {
	if (isPort())
		return getPosition(dim);
	return getBounds().computeCenter(dim);
} // end method

// -----------------------------------------------------------------------------

inline const Bounds &Instance::getBounds() const {
	return data->clsBounds;
} // end method

// -----------------------------------------------------------------------------

inline PhysicalOrientation Instance::getOrientation() const {
	return data->clsOrientation;
} // end method

// -----------------------------------------------------------------------------

inline PhysicalTransform Instance::getTransform(const bool origin) const {
	if (origin) {
		Bounds bounds = getBounds();
		bounds.translate(-bounds.getLower());
		return PhysicalTransform(bounds, getOrientation());
	} else {
		return PhysicalTransform(getBounds(), getOrientation());
	} // end else
} // end method

// =============================================================================
// Tag
// =============================================================================

inline
TristateFlag
InstanceTag::getFixed() const {
	return data->fixed;
} // end method

// -----------------------------------------------------------------------------

inline
void
InstanceTag::setFixed(const bool value) {
	data->fixed = value;
} // end method

// -----------------------------------------------------------------------------

inline
TristateFlag
InstanceTag::getMacroBlock() const {
	return data->block;
} // end method

// -----------------------------------------------------------------------------

inline
void
InstanceTag::setMacroBlock(const bool value) {
	data->block = value;
} // end method

} // end namespace
