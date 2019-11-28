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

//! @brief A proxy class representing a netlist net.
class Net : public Proxy<NetData> {
	
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;
friend class Module;
friend class Pin;
friend class Arc;

private:
	Net(NetData * data) : Proxy(data) {}
	
	Design _getDesign() const;
	Module _getParent() const;			
	
public:

	//! @brief Default constructor.
	Net() {}

	//! @brief Assignment constructor to allow null values.
	Net(std::nullptr_t) {}

	//! @brief Returns the design in which this net was instantiated.
	Design getDesign();

	//! @brief Returns the design in which this net was instantiated.
	const Design getDesign() const;

	//! @brief Returns the parent module in which this net was instantiated.
	Module getParent();

	//! @brief Returns the parent module in which this net was instantiated.
	const Module getParent() const;

	//! @brief Returns the name of this net.
	const std::string &getName() const;

	//! @brief Returns the number of pins connected to this net.
	int getNumPins() const;

	//! @brief Returns the number of sinks (input) pins connected to this net.
	int getNumSinks() const;

	//! @brief Returns the number of drivers (output) pins connected to this
	//!        net.
	int getNumDrivers() const;

	//! @brief Returns any driver pin if any. Useful when there's an assumption
	//!        that nets have a single driver (most common case).
	Pin getAnyDriver() const;

	//! @brief Returns the arc connecting the "from" pin to the "to" pin if any.
	//! @note  Net arcs only exist from driver to sink pins.
	Arc getArc(const Pin from, const Pin to);

	//! @brief Returns the topological index of this net.
	//! @note  The topological index of a net is the maximum topological index
	//!        of its drivers. If no drivers, it's the maximum topological index
	//!        of its sinks. If this is a floating net (no pins), returns the
	//!        constant value Rsyn::MIN_TOPOLOGICAL_INDEX.
	TopologicalIndex getTopologicalIndex() const;

	//! @brief Returns true if this net has more than one driver pin and false
	//!        otherwise.
	bool hasMultipleDrivers() const;

	//! @brief Returns true if this net has only and only one driver and false
	//!        otherwise.
	bool hasSingleDriver() const;

	//! @brief Returns true if this net has at least one driver and false
	//!        otherwise.
	bool hasDriver() const;

	//! @brief Returns true if this net has at least one sink and false
	//!        otherwise.
	bool hasSink() const;

	//! @brief Returns an iterable collection of the pins connected to this
	//!        net.
	Range<CollectionOfPins>
	allPins(bool filterPG = true) const;

	//! @brief Returns an iterable collection of the pins of a given direction
	//!        connected to this net.
	Range<CollectionOfPinsFilteredByDirection>
	allPins(const Direction direction) const;

	//! @brief Returns an iterable collection of the arcs spanned by this net.
	Range<CollectionOfArcs>
	allArcs() const;	

	//! @brief Returns true if this net should be considered as ideal and false
	//!        otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isIdeal() const;

	//! @brief Returns true if this net represents a clock network.
	//! @note  A clock network net does not need to be directly connected to the
	//!        clock port. Internal nets of a clock buffer tree should also be
	//!        considered as clock networks.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isClockNetwork() const;

	//! @brief Returns the type ot this net.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	NetTypeTag getNetTypeTag() const;

	//! @Author Mateus
	//! @brief Returns the usage of the net
	Use getUse() const;
	
	//! @Author Mateus
	//! @brief Returns the usage of the net
	void setUse(const Use use);
}; // end class

// =============================================================================
// Tag
// =============================================================================

//! @brief TODO
class NetTag : public Proxy<NetTagData> {
friend class Design;
private:
	NetTag(NetTagData * data) : Proxy(data) {}

public:

	NetTag() {}
	NetTag(std::nullptr_t) {}

	NetTypeTag getType() const;
	TristateFlag getIdeal() const;

	void setType(const NetTypeTag value);
	void setIdeal(const bool value);
}; // end class

} // end namespace