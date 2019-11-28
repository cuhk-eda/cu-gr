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

//! @brief A proxy class representing a netlist pin.
class Pin : public Proxy<PinData> {	
	
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;
friend class Port;
friend class Net;

private:
	Pin(PinData * data) : Proxy(data) {}
	
public:

	//! @brief Default constructor.
	Pin() {}

	//! @brief Assignment constructor to allow null values.
	Pin(std::nullptr_t) {}	

	//! @brief Returns the design in which this pin is instantiated.
	Design getDesign();

	//! @brief Returns the design in which this pin is instantiated.
	const Design getDesign() const;	

	//! @brief Returns the direction of this pin.
	Direction getDirection() const;

	//! @brief Returns the index of this pin relative to its instance and
	//! library cell.
	int getIndex() const;

	//! @brief Returns the name of this pin.
	const std::string getName() const;

	//! @brief Returns the full name of this pin (e.g. nand:a).
	const std::string getFullName(const std::string::value_type separator = ':') const;

	//! @brief Returns the name of the instance which this pin belongs to.
	const std::string getInstanceName() const;

	//! @brief Returns the name of the net which this pin connects to if any.
	const std::string &getNetName() const;

	//! @brief Returns the direction name of this pin.
	const std::string &getDirectionName() const;

	//! @brief Returns the net which this pin connects to if any.
	Net getNet() const;

	//! @brief Returns the instance which this pin belongs to.
	Instance getInstance() const;

	//! @brief Returns the type of the instance which this pin belongs to.
	InstanceType getInstanceType() const;

	//! @brief Returns the library pin associate to this pin if any.
	//! @note  Only pins from cells are associated to library pins.
	LibraryPin getLibraryPin() const;

	//! @brief Returns the library cell associate to this pin if any.
	//! @note  Only pins from cells are associated to library cells.
	LibraryCell getLibraryCell() const;

	//! @brief If this pin belongs to a port instance, returns the port and
	//!        null otherwise.
	Port getPort() const;

	//! @brief Returns the topological index of this pin.
	TopologicalIndex getTopologicalIndex() const;

	//! @brief Returns true if this pin belongs to a port instance.
	bool isPort() const;

	//! @brief Returns true if this pin belongs to a port instance of a given
	//!        direction.
	bool isPort(const Direction direction) const;

	//! @brief Returns true if this pin belongs to a port that connects to an
	//!        up-level hierarchy and false otherwise.
	bool isPortToUpLevelHierarchy() const;

	//! @brief Returns true if this pin belongs to a port that connects to an
	//!        down-level hierarchy and false otherwise.
	bool isPortToDownLevelHierarchy() const;

	//! @brief Returns true if this pin is connected to a net and false
	//!        otherwise.
	bool isConnected() const;

	//! @brief Returns true if this pin is not connected to a net and false
	//!        otherwise.
	bool isDisconnected() const;

	//! @brief Returns true if this pin is an input pin and false otherwise.
	bool isInput() const;

	//! @brief Returns true if this pin is an output pin and false otherwise.
	bool isOutput() const;

	//! @brief Returns true if this pin is a bidirectional pin and false
	//!        otherwise.
	bool isBidirectional() const;	

	//! @brief Returns true if this pin is a driver (output) pin and false
	//!        otherwise.
	bool isDriver() const;

	//! @brief Returns true if this pin is a sink (input) pin and false
	//!        otherwise.
	bool isSink() const;	

	//! @brief Returns the arc from this pin to the "to" pin if any.
	Arc getArcTo(Pin to);

	//! @brief Returns the arc from the "from" pin to this pin if any.
	Arc getArcFrom(Pin from);

	//! @brief Returns the number of arcs that have this pin as the "to" pin.
	int getNumIncomingArcs() const;

	//! @brief Returns the number of arcs that have this pin as the "from" pin.
	int getNumOutgomingArcs() const;

	//! @brief Returns an iterable collection of all pins that are directly
	//!        connected to this pin via incoming arcs (i.e. all "from" pins
	//!        of the arcs that have this pin as the "to" pin).
	Range<CollectionOfPredecessorPins>
	allPredecessorPins(const bool crossBoundaries = false) const;

	//! @brief Returns an iterable collection of all pins that are directly
	//!        connected to this pin via outgoing arcs (i.e. all "to" pins
	//!        of the arcs that have this pin as the "from" pin).
	Range<CollectionOfSuccessorPins>
	allSucessorPins(const bool crossBoundaries = false) const;	

	//! @brief Returns an iterable collection of all arcs that have this pin as
	//!        the "to" pin.
	const std::vector<Arc> &
	allIncomingArcs() const;

	//! @brief Returns an iterable collection of all arcs that have this pin as
	//!        the "from" pin.
	const std::vector<Arc> &
	allOutgoingArcs() const;

	//! @brief Returns an iterable collection of all arcs given a traverse
	//!        direction.
	const std::vector<Arc> &
	allArcs(const TraverseType direction) const;

	//! @brief Connects this pin to a net. If the pin is currently connected,
	//!        it will be first disconnected.
	void connect(Net net);

	//! @brief Disconnect this pin.
	void disconnect();	

	//! @brief Returns true if this pin belongs to a non-standard-cell and false
	//!        otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isMacroBlockPin() const;

	//! @brief Returns true if this pin is connected to a clock network and
	//!        false otherwise.
	//! @note  This should be true even if the pin is indirectly connected to
	//!        the clock (i.e. via buffers).
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isConnectedToClockNetwork() const;

	//! @Author Mateus
	//! @brief Returns the usage of the pins
	Use getUse() const;
	
	//! @Author Mateus 
	//! @brief Returns true if the pin is used for power or ground or false
	//!        otherwise.
	bool isPowerOrGround() const;
}; // end class
	
} // end namespace