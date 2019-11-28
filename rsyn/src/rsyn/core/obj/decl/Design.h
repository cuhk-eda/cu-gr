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

// -----------------------------------------------------------------------------

//! @brief A proxy class representing the design netlist.
class Design : public Proxy<DesignData> {
friend class RawPointer;

friend class Session;
friend class Library;
friend class Net;
friend class Cell;
friend class Pin;
friend class Instance;
friend class Module;

friend class Sandbox;
friend class SandboxNet;
friend class SandboxInstance;

template<typename _Object, typename _ObjectReference, typename _ObjectExtension> friend class AttributeBase;
template<typename _Object, typename _ObjectExtension> friend class AttributeImplementation;

private:

	Design(DesignData * data) : Proxy(data) {}

	//! @brief Create a new empty design.
	void create(const std::string &name);

public:

	//! @brief Default constructor.
	Design() {}

	//! @brief Assignment constructor to allow null values.
	Design(std::nullptr_t) {}

	//! @brief Creates a new designs.
	Design(const std::string &name);

	//! @brief Copy operator.
	//! @note  Not sure if this is actually necessary. I don't remember why it
	//!        was created.
	Design &operator=(const Design &other) {
		data = other.data;
		return *this;
	} // end method	

	//! @brief Returns the top module of the design.
	Module getTopModule();

	//! @brief Returns the name of the design.
	const std::string &getName() const;

	//! @brief Changes the name of the design.
	void updateName(std::string name);
	
private:

	//! @brief A default string to represent the name of null objects.
	static const std::string NULL_NAME;

	//! @brief Generates an unique name for a new instance.
	std::string generateUniqueInstanceName(const std::string &prefix);

	//! @brief Generates an unique name for a new net.
	std::string generateUniqueNetName(const std::string &prefix);

	//! @brief Generates the next sign to be used in some traversal algorithms
	//!        as topological sorting.
	int generateNextSign();

	//! @brief Gets the current sign to be used in some traversal algorithms as
	//!        topological sorting.
	int getSign() const;
	
	////////////////////////////////////////////////////////////////////////////
	// Unique Identifiers for Rsyn Objects
	//--------------------------------------------------------------------------
	// Currently Rsyn uses unique identifiers (indexes) internally to manage 
	// layers.
	//
	// Indexes should not be exposed to users as they are merely an internal
	// way to support layers and this scheme may be changed in the future.
	//
	// Layers allow users to associate data to the several Rsyn objects.
	// Moreover layers are aware to incremental changes in the netlist.
	////////////////////////////////////////////////////////////////////////////	
private:

	//! @brief Gets the internal id of a net.
	Index getId(Net net) const;

	//! @brief Gets the internal id of an instance.
	Index getId(Instance instance) const;

	//! @brief Gets the internal id of a pin.
	Index getId(Pin pin) const;

	//! @brief Gets the internal id of an arc.
	Index getId(Arc arc) const;

	//! @brief Gets the internal id of a library cell.
	Index getId(LibraryCell lcell) const;

	//! @brief Gets the internal id of a library pin.
	Index getId(LibraryPin lpin) const;

	//! @brief Gets the internal id of a library arc.
	Index getId(LibraryArc larc) const;
	
	////////////////////////////////////////////////////////////////////////////
	// Library
	////////////////////////////////////////////////////////////////////////////	
public:

	//! @brief Creates a new library cell.
	LibraryCell createLibraryCell(const CellDescriptor &dscp, const bool ignoreDuplicated = false);
	
	////////////////////////////////////////////////////////////////////////////
	// Netlist
	////////////////////////////////////////////////////////////////////////////
private:
	
	//! @brief Instantiates a new cell inside a module.
	Cell createCell(const Module parent, const LibraryCell lcell, const std::string &name);

	//! @brief Instantiates a new port inside a module.
	Port createPort(Module parent, const Direction direction, const std::string &name);

	//! @brief Creates a new module.
	Module createModule(const LibraryModule lmoudle, const std::string &name);

	//! @brief Instantiates a new net inside a module.
	Net createNet(Module parent, const std::string &name);

	//! @brief Connects a pin to a net.
	void connectPin(Pin pin, Net net);

	//! @brief Disconnect a pin.
	void disconnectPin(Pin pin);

	//! @brief Changes the library cell of a cell. The new library cell must
	//!        have the same interface (i.e. pin names and directions) as the
	//!        old one.
	void remap(Cell cell, LibraryCell newLibraryCell);

	//! @brief Changes the library cell of a cell. The new library cell must
	//!        have the same interface (i.e. pin names and directions) as the
	//!        old one.
	void remap(Cell cell, const std::string &newLibraryCellName);

public:

	//! @brief Gets the current number of instances in the design.
	int getNumInstances() const;

	//! @brief Gets the current number of instances of a given type in the
	//!        design.
	int getNumInstances(const InstanceType type) const;

	//! @brief Gets the current number of nets in the design.
	int getNumNets() const;

	//! @brief Gets the current number of pins in the design.
	int getNumPins() const;
        
        Range<ReferenceListCollection<Cell>> getAllCells() const;
	
	////////////////////////////////////////////////////////////////////////////
	// Topological Ordering
	////////////////////////////////////////////////////////////////////////////
private:

	//! @brief Updates incrementally the topological ordering given a change in
	//!        a pin (e.g. pin gets connected).
	void updateTopologicalIndex(Pin pin);
	
	////////////////////////////////////////////////////////////////////////////
	// Events
	////////////////////////////////////////////////////////////////////////////	
public:

	//! @brief Registers an observer to be notified about changes in the
	//!        netlist.
	template<class T>
	void
	registerObserver(T *observer);

	//! @brief Unregisters an observer so it will no longer receives
	//!        notifications about changes in the netlist.
	void
	unregisterObserver(DesignObserver *observer);

	//! @brief Notifies about a change in an instance placement. Usually this
	//! does not need to be done manually, but notification can be turned off
	//! and then observer must be manually notified.
	//! @todo Return also the old position.
	void
	notifyInstancePlaced(Rsyn::Instance instance, Rsyn::DesignObserver *ignoreObserver = nullptr);

	////////////////////////////////////////////////////////////////////////////
	// Searching
	////////////////////////////////////////////////////////////////////////////	
public:

	//! @brief Finds an instance by name. If the instance is not found, returns
	//!        null.
	Instance findInstanceByName(const std::string &name) const;

	//! @brief Finds a cell by name. If the cell is not found, returns null.
	Cell findCellByName(const std::string &name) const;

	//! @brief Finds a port by name. If the port is not found, returns null.
	Port findPortByName(const std::string &name) const;

	//! @brief Finds a module by name. If the module is not found, returns
	//!        null.
	Module findModuleByName(const std::string &name) const;

	//! @brief Finds a library cell by name. If the library cell is not found,
	//!        returns null.
	LibraryCell findLibraryCellByName(const std::string &name) const;

	//! @brief Finds a net by name. If the net is not found, returns null.
	Net findNetByName(const std::string &name) const;

	//! @brief Finds a pin by name. If the pin is not found, returns null.
	//! @note  A pin name is in the form <instance>:<pin name>.
	Pin findPinByName(const std::string &cellName, const std::string &pinName) const;

	//! @brief Finds a pin by name. If the pin is not found, returns null.
	//! @note  A pin name is in the form <instance>:<pin name>.
	Pin findPinByName(const std::string &name, const std::string::value_type separator = ':') const;	
	
	////////////////////////////////////////////////////////////////////////////
	// Attributes
	////////////////////////////////////////////////////////////////////////////
public:

	//! @brief Creates an object attribute.
	AttributeInitializer 
	createAttribute();

	//! @brief Creates an object attribute with a default value.
	template<typename DefaultValueType>
	AttributeInitializerWithDefaultValue<DefaultValueType>
	createAttribute(const DefaultValueType &defaultValue);

	////////////////////////////////////////////////////////////////////////////
	// User Flags
	////////////////////////////////////////////////////////////////////////////
public:

	//! @brief Gets the tag information associate to a net.
	NetTag getTag(Rsyn::Net net);

	//! @brief Gets the tag information associate to an instance.
	InstanceTag getTag(Rsyn::Instance instance);

	//! @brief Gets the tag information associate to a library cell.
	LibraryCellTag getTag(Rsyn::LibraryCell libraryCell);
	
	////////////////////////////////////////////////////////////////////////////
	// Range-Based Loops
	////////////////////////////////////////////////////////////////////////////

	//! @brief Returns an iterable collection of all library cells in the
	//!        design.
	//! @todo  This should be removed once we concentrate everything library
	//!        related in Rsyn::Library.
	Range<ListCollection<LibraryCellData, LibraryCell>>
	allLibraryCells(const bool showDeprecatedMessage = true);
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Range-Based Loop Collections
////////////////////////////////////////////////////////////////////////////////

//! @brief TODO
template<class Object, class Reference, unsigned int CHUNK_SIZE>
class GenericListCollection {
protected:
	ConstList<Object, CHUNK_SIZE> list;
	typename ConstList<Object, CHUNK_SIZE>::Iterator it;
public:
	GenericListCollection(const List<Object, CHUNK_SIZE> &pins)
			: list(pins), it(list.begin()) {}

	bool filter() { return false; }
	bool stop() { return it.stopFlag(); }
	void next() { ++it; }
	Reference current() { return &((*it).getPointer()->value); } // TODO: awful
}; // end class	

// -------------------------------------------------------------------------

//! @brief TODO
template<class Reference, unsigned int CHUNK_SIZE>
class GenericReferenceListCollection {
protected:
	ConstList<Reference, CHUNK_SIZE> list;
	typename ConstList<Reference, CHUNK_SIZE>::Iterator it;
public:
	GenericReferenceListCollection(const List<Reference, CHUNK_SIZE> &pins)
			: list(pins), it(list.begin()) {}

	bool filter() { return false; }
	bool stop() { return it.stopFlag(); }
	void next() { ++it; }
	Reference current() { return (*it).getPointer()->value; } // TODO: awful
}; // end class	

// -------------------------------------------------------------------------

//! @brief TODO
template<class PinType>
class CollectionOfGenericPinsFilteredByDirection {
protected:
	std::vector<PinType> &pins;
	Direction direction;
	int index;
	bool filterPG;
public:
	CollectionOfGenericPinsFilteredByDirection(std::vector<PinType> &pins,
			Direction direction, bool filterPG = true) 
			: pins(pins), direction(direction), index(0), filterPG(filterPG) {}

	bool filter() { return current().getDirection() != direction || 
			(filterPG && current().isPowerOrGround()); }
	bool stop() { return index >= pins.size(); }
	void next() { ++index; }
	PinType current() {	return pins[index];	}
}; // end class

// -------------------------------------------------------------------------

//! @brief TODO
template<class PinType>
class CollectionOfGenericPins {
protected:
	std::vector<PinType> &pins;
	int index;
	bool filterPG;
public:
	CollectionOfGenericPins(std::vector<PinType> &pins, bool filterPG = true)
			: pins(pins), index(0), filterPG(filterPG) {}

	bool filter() { return filterPG && current().isPowerOrGround(); }
	bool stop() { return index >= pins.size(); }
	void next() { ++index; }
	PinType current() {	return pins[index];	}
}; // end class

// -------------------------------------------------------------------------

//! @brief TODO
class CollectionOfLibraryPinsFilteredByDirection {
protected:
	std::vector<LibraryPin> &pins;
	Direction direction;
	int index;
public:
	CollectionOfLibraryPinsFilteredByDirection( std::vector<LibraryPin> &pins,
			Direction direction) 
			: pins(pins), direction(direction), index(0) {}

	bool filter() { return current().getDirection() != direction; }
	bool stop() { return index >= pins.size(); }
	void next() { ++index; }
	LibraryPin current() {	return pins[index];	}
}; // end class	

// -------------------------------------------------------------------------

//! @brief TODO
class CollectionOfLibraryPins {
protected:
	std::vector<LibraryPin> &pins;
	int index;
public:
	CollectionOfLibraryPins(std::vector<LibraryPin> &pins) 
			: pins(pins), index(0) {}

	bool filter() { return false; }
	bool stop() { return index >= pins.size(); }
	void next() { ++index; }
	LibraryPin current() {	return pins[index];	}
}; // end class	

// -------------------------------------------------------------------------

//! @brief TODO
template<class ArcType>
class CollectionOfGenericArcs {
protected:
	std::vector<ArcType> &arcs;
	int index;
public:
	CollectionOfGenericArcs(std::vector<ArcType> &arcs)
			: arcs(arcs), index(0) {}

	bool filter() { return false; }
	bool stop() { return index >= arcs.size(); }
	void next() { ++index; }
	ArcType current() { return arcs[index]; }
}; // end class

// -------------------------------------------------------------------------

//! @brief TODO
class CollectionOfLibraryArcs {
protected:
	std::vector<LibraryArc> &arcs;
	int index;
public:
	CollectionOfLibraryArcs(std::vector<LibraryArc> &arcs) 
			: arcs(arcs), index(0) {}

	bool filter() { return false; }
	bool stop() { return index >= arcs.size(); }
	void next() { ++index; }
	LibraryArc current() {	return arcs[index];	}
}; // end class

// -------------------------------------------------------------------------

//! @brief TODO
class CollectionOfLibraryArcsToLibraryPin {
protected:
	const std::vector<LibraryArc> &arcs;
	LibraryPin to;
	size_t index;
public:
	CollectionOfLibraryArcsToLibraryPin(const std::vector<LibraryArc> &arcs, LibraryPin to) 
	: arcs(arcs), to(to) {
		index = 0;
	} // end constructor

	bool filter() { return arcs[index].getToLibraryPin() != to; }
	bool stop() { return index >= arcs.size(); }
	void next() { index++; }
	LibraryArc current() { return arcs[index]; }
}; // end class

// -------------------------------------------------------------------------

//! @brief TODO
class CollectionOfLibraryArcsFromLibraryPin {
protected:
	const std::vector<LibraryArc> &arcs;
	LibraryPin from;
	size_t index;
public:
	CollectionOfLibraryArcsFromLibraryPin(const std::vector<LibraryArc> &arcs, LibraryPin from) 
	: arcs(arcs), from(from) {
		index = 0;
	} // end constructor

	bool filter() { return arcs[index].getFromLibraryPin() != from; }
	bool stop() { return index >= arcs.size(); }
	void next() { index++; }
	LibraryArc current() { return arcs[index]; }
}; // end class		

// -------------------------------------------------------------------------

//! @brief TODO
//! @todo  Update this when net arcs are implemented.
template<class PinType, class NetType, class ArcType>
class CollectionOfGenericPredecessorPins {
protected:
	std::vector<PinType> pins;
	int index;
public:
	
	CollectionOfGenericPredecessorPins(PinType pin, const bool crossBoundary) {
		switch (pin.getDirection()) {
		case IN: {
			NetType net = pin.getNet();
			if (net) {
				pins.reserve(net.getNumDrivers());
				for (PinType predecessor : net.allPins(DRIVER)) {
					pins.push_back(predecessor);
				} // end for
			} // end if
			break;
		} // end case

		case OUT: {
			if (pin.isPort()) {
				// An output pin means an input port.
				if (crossBoundary) {
					PinType related = pin.getPort().getOtherPin(pin);
					if (related) {
						pins.push_back(related);
					} // end if				
				} // end if					
			} else {
				for (ArcType arc : pin.allIncomingArcs()) {
					pins.push_back(arc.getFromPin());
				} // end for
			}
			break;
		} // end case

		default:
			std::cout << "[ERROR] 1e1673fccd9b42d225b502a273cb3a20\n";
			std::exit(1);
		} // end switch
		
		index = 0;
	} // end constructor

	bool filter() { return false; }
	bool stop() { return index >= pins.size(); }
	void next() { ++index; }
	PinType current() {	return pins[index];	}
}; // end class	

// -------------------------------------------------------------------------

//! @brief TODO
//! @todo  Update this when net arcs are implemented.
template<class PinType, class NetType, class ArcType>
class CollectionOfGenericSuccessorPins {
protected:
	std::vector<PinType> pins;
	int index;
public:
	CollectionOfGenericSuccessorPins(PinType pin, const bool crossBoundary) {
		
		switch (pin.getDirection()) {
		case IN: {
			if (pin.isPort()) {
				// An input pin means an output port.
				if (crossBoundary) {
					PinType related = pin.getPort().getOtherPin(pin);
					if (related) {
						pins.push_back(related);
					} // end if				
				} // end if					
			} else {
				for (ArcType arc : pin.allOutgoingArcs()) {
					pins.push_back(arc.getToPin());
				} // end for
			} // end else
			break;
		} // end case

		case OUT: {
			NetType net = pin.getNet();
			if (net) {
				pins.reserve(net.getNumSinks());
				for (PinType successor : net.allPins(SINK)) {
					pins.push_back(successor);
				} // end for
			} // end if
			break;
		} // end case

		default:
			std::cout << "[ERROR] 44167c065ebd3a23a553fdd96301831e\n";
			std::exit(1);
		} // end switch				
	
		index = 0;
	} // end constructor

	bool filter() { return false; }
	bool stop() { return index >= pins.size(); }
	void next() { ++index; }
	PinType current() { return pins[index];	}
}; // end class

} // end namespace