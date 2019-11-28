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
 
namespace Rsyn {

//! @brief A proxy class representing a netlist instance (i.e. cell, port or
//!        module).
class Instance : public Proxy<InstanceData> {
	
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;
friend class Pin;
friend class Cell;
friend class Port;
friend class Module;

// Temporary while we merge some physical data into the netlist (2018/01/17).
friend class PhysicalDesign;
friend class PhysicalService;

private:
	Instance(InstanceData * data) : Proxy(data) {}

	DesignData * _getDesignData() const;
	Design _getDesign() const;
	Module _getParent() const;

	// TODO: remove, use -> instead
	DesignData * getDesignData();
	const DesignData * getDesignData() const;
	
public:

	//! @brief Default constructor.
	Instance() {}

	//! @brief Assignment constructor to allow null values.
	Instance(std::nullptr_t) {}	

	//! @brief Converts this instance to a cell.
	Cell asCell() const;

	//! @brief Converts this instance to a port.
	Port asPort() const;

	//! @brief Converts this instance to a module.
	Module asModule() const;

	//! @brief Returns the design in which this instance is instantiated.
	Design getDesign();
	
	//! @brief Returns the design in which this instance is instantiated.
	const Design getDesign() const;

	//! @brief Returns the parent module in which this instance is instantiated.
	Module getParent();

	//! @brief Returns the parent module in which this instance is instantiated.
	const Module getParent() const;

	//! @brief Returns the type of this instance.
	InstanceType getType() const;

	//! @brief Returns the name of this instance.
	const std::string &getName() const;

	//! @brief Returns the hierarchical name of this instance.
	std::string getHierarchicalName() const;

	//! @brief Returns the number of interface pin.
	int getNumPins() const;

	//! @brief Returns the number of interface pin of a given direction.
	int getNumPins(const Direction direction) const;

	//! @brief Returns the number of interface input pin.
	int getNumInputPins() const;

	//! @brief Returns the number of interface output pin.
	int getNumOutputPins() const;

	//! @brief Returns the number of interface bidirectional pin.
	int getNumBidirectionalPins() const;

	//! @brief Returns the number of arcs.
	int getNumArcs() const;

	//! @brief Returns a pin by its index.
	Pin getPinByIndex(const int index) const;

	//! @brief Returns a pin by its name. If no pin matches, null is returned.
	Pin getPinByName(const std::string &name) const;

	//! @brief Returns any input pin. Useful when handling buffers/inverters,
	//!        which have only one input pin.
	Pin getAnyInputPin() const;

	//! @brief Returns any output pin. Useful when handling buffers/inverters,
	//!        which have only one output pin.
	Pin getAnyOutputPin() const;

	//! @brief Returns the arc from the "from" pin to the "to" pin. Returns null
	//!        if the pins do not belong to this instance or if no such arc is
	//!        found.
	Arc getArc(const Pin from, const Pin to);

	//! @brief Returns the arc from the "from" pin to the "to" pin. Returns null
	//!        if either pin is not found or if no such arc exist between the
	//!        pins.
	Arc getArcByPinNames(const std::string &from, const std::string &to);

	//! @brief Returns any arc. Useful when handling buffers/inverters, which
	//!        have only one arc.
	Arc getAnyArc();

	//! @brief Returns the library cell associated to this instance if any.
	LibraryCell getLibraryCell() const;

	//! @brief Returns the topological index of this instance.
	//! @note  Since the topological index of an instance may be ambiguous (e.g.
	//!        for register), some choice needed to be made here. Currently,
	//!        the topological index of an instance is the maximum topological
	//!        index of the "from" pins of its arcs. If no arcs, the topological
	//!        index is set to the maximum topological index of all pins.
	//!        In this way, the topological index of registers is given by their
	//!        clock pin rather than by their data pin.
	TopologicalIndex getTopologicalIndex() const;

	//! @brief Returns true if this instance represents a port.
	bool isPort() const;

	//! @brief Returns true if this instance represents a port and the port
	//!        matches the given direction.
	bool isPort(const Direction direction);

	//! @brief Returns an iterable collection of all interface pins of a given
	//!        direction.
	//! @note  The name of this method may be confusing for modules where one
	//!        may expect the collection of all pins inside the module. However
	//!        to be consistent, only the interface pins are returned that is
	//!        the outer pins of the module's ports.
	Range<CollectionOfPinsFilteredByDirection>
	allPins(const Direction direction, bool filterPG = true) const;

	//! @brief Returns an iterable collection of all interface pins.
	//! @see allPins(const Direction direction)
	Range<CollectionOfPins>
	allPins(bool filterPG = true) const;	

	//! @brief Returns an iterable collection of all arcs.
	Range<CollectionOfArcs>
	allArcs() const;

	//! @brief Returns true if this instance represents a sequential cell and
	//!        false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isSequential() const;

	//! @brief Returns true if this instance represents a tie cell and false
	//!        otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isTie(const TieType type) const;

	//! @brief Returns true if this instance represents a buffer cell of a given
	//!        type and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isBuffer(const BufferType type) const;

	//! @brief Returns true if this instance represents a buffer cell of a given
	//!        type, which is driven by a clock network and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isClockBuffer(const BufferType type = ANY_BUFFER_TYPE, const bool local = false) const;

	//! @brief Returns true if this instance represents a local clock buffer of
	//!        a given type and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isLCB(const BufferType type = ANY_BUFFER_TYPE) const;

	//! @brief Returns true if this instance cannot be moved (i.e. has its
	//!        position fixed) and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isFixed() const;

	//! @brief Returns true if this instance can be moved and false otherwise.
	//! @see   isFixed()
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isMovable() const;

	//! @brief Returns true if this instance represents a non-standard-cell and
	//!        false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isMacroBlock() const;

	//! @brief Returns true if this instance orientation is flipped.
	//! @see   isFlipped()
	bool isFlipped() const;
	
	//! @brief Returns the x position (lower-left) of the instance.
	DBU getX() const;

	//! @brief Returns the y position (lower-left) of the instance.
	DBU getY() const;

	//! @brief Returns the height of this instance.
	DBU getHeight() const;

	//! @brief Returns the width of this instance.
	DBU getWidth() const;

	//! @brief Returns the size of this instance.
	//! @note  X is the length for abscissa (width) while in Y is the length for
	//!        ordinate (height).
	DBUxy getSize() const;

	//! @brief Returns the size of this instance.
	//! @todo Rename to getSize().
	DBU getSize(const Dimension dimension) const;

	//! @brief Returns the area of this instance.
	DBU getArea() const;

	//! @brief Returns the lower-left position of this instance.
	DBUxy getPosition() const;

	//! @brief Returns the lower-left position of this instance.
	//! that is defined as its position.
	DBU getPosition(const Dimension dim) const;

	//! @brief Returns the lower-left/upper-right position of this instance.
	//! @todo remove
	DBUxy getCoordinate(const Boundary bound) const;

	//! @brief Returns the lower-left/upper-right position of this instance.
	//! @todo remove
	DBU getCoordinate(const Boundary bound, const Dimension dim) const;

	//! @brief Returns the central point of the PhysicalInstance boundaries.
	DBUxy getCenter() const;

	//! @brief Returns the center position in a given dimension.
	DBU getCenter(const Dimension dim) const;

	//! @brief Returns the orientation of the cell.
	PhysicalOrientation getOrientation() const;

	//! @brief Returns a transformation that allows one to transform the
	//! coordinates from the library cell space to the cell space. The
	//! transformation accounts for translation and the orientation of the cell.
	//! @param origin If set to true, the transform is created w.r.t (0, 0) and
	//! not the current cell position.
	PhysicalTransform getTransform(const bool origin = false) const;

	//! @brief Returns the bound box Bounds that defines the limits of PhysicalInstance.
	const Bounds &getBounds() const;

}; // end class

// =============================================================================
// Tag
// =============================================================================

//! @brief TODO
class InstanceTag : public Proxy<InstanceTagData> {
friend class Design;
private:	
	InstanceTag(InstanceTagData * data) : Proxy(data) {}
	
public:
	
	InstanceTag() {}
	InstanceTag(std::nullptr_t) {}

	TristateFlag getFixed() const;
	TristateFlag getMacroBlock() const;

	void setFixed(const bool value);
	void setMacroBlock(const bool value);
}; // end class

} // end namespace
