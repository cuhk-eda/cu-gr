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

//! @brief A proxy class representing a netlist module.
class Module : public Instance {
	
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;
friend class Instance;

private:
	Module(InstanceData * data) : Instance(data) {}

	//! @brief Generates the next sign to be used in some traversal algorithms
	//!        as topological sorting.
	int generateNextSign() const;

	//! @brief Gets the current sign to be used in some traversal algorithms as
	//!        topological sorting.
	int getSign() const;

	//! @brief This method is inherited from Instance. Hide to avoid confusion
	//!        with internal pins. See allInterfacePins() and allInternalPins().
	Range<CollectionOfPinsFilteredByDirection>
	allPins(const Direction direction) const;

	//! @brief This method is inherited from Instance. Hide to avoid confusion
	//!        with internal pins. See allInterfacePins() and allInternalPins().
	Range<CollectionOfPins>
	allPins() const;

	//! @brief This method is inherited from Instance. Hide to avoid confusion
	//!        with internal pins. See allInterfaceArcs() and allInternalArcs().
	Range<CollectionOfArcs>
	allArcs() const;

public:

	//! @brief Default constructor.
	Module() : Instance(nullptr) {}

	//! @brief Assignment constructor to allow null values.
	Module(std::nullptr_t) : Instance(nullptr) {}

	//! @brief Returns the design in which this module is instantiated.
	Design getDesign();

	//! @brief Returns the design in which this module is instantiated.
	const Design getDesign() const;

	//! @brief Creates a new cell in this module. If no name is provided, an
	//!        automatically generated name will be used.
	Cell createCell(const std::string &libraryCellName, const std::string &cellName = "");

	//! @brief Creates a new cell in this module. If no name is provided, an
	//!        automatically generated name will be used.
	Cell createCell(const LibraryCell lcell, const std::string &name = "");

	//! @brief Creates a new port in this module. If no name is provided, an
	//!        automatically generated name will be used.
	Port createPort(const Direction &direction, const std::string &name = "");

	//! @brief Creates a new net in this module. If no name is provided, an
	//!        automatically generated name will be used.
	Net createNet(const std::string &name = "");

	//! @brief Returns a port by its index.
	Port getPortByIndex(const int index);

	//! @brief Constructs a vector of instances stratified by logical depth such
	//!        that all cells with logical depth "i" will be at index "i".
	//! @note  See Instance::getTopologicalIndex() description to check how
	//!        the topological index of instances is defined.
	void getInstancesPerLogicalDepth(std::vector<std::vector<Rsyn::Instance>> &levels);

	//! @brief Constructs a vector of nets stratified by logical depth such
	//!        that all nets with logical depth "i" will be at index "i".
	//! @note  See Net::getTopologicalIndex() description to check how
	//!        the topological index of nets is defined.
	void getNetsPerLogicalDepth(std::vector<std::vector<Rsyn::Net>> &levels);

	//! @brief Returns the number of ports of a given directions.
	int getNumPorts(const Direction direction) const;

	//! @brief Returns an iterable collection of all instances instantiated in
	//!        this module.
	Range<ReferenceListCollection<Instance>>
	allInstances() const;

	//! @brief Returns an iterable collection of all ports instantiated in this
	//!        module.
	Range<ReferenceListCollection<Port>>
	allPorts() const;

	//! @brief Returns an iterable collection of all ports of a given direction
	//!        inside this module.
	std::set<Port> &
	allPorts(const Rsyn::Direction direction) const;

	//! @brief Returns an iterable collection of all nets instantiated in this
	//!        module.
	Range<ReferenceListCollection<Net>>
	allNets() const;

	//! @brief Returns an iterable collection of all interface pins of a given
	//!        direction.
	Range<CollectionOfPinsFilteredByDirection>
	allInterfacePins(const Direction direction) const;

	//! @brief Returns an iterable collection of all interface pins (i.e. outer
	//!        pin of ports.
	//! @see   allInterfacePins(const Direction direction)
	Range<CollectionOfPins>
	allInterfacePins() const;

	//! @brief Returns an iterable collection of all arcs.
	Range<CollectionOfArcs>
	allInterfaceArcs() const;

	//! @brief Returns an iterable collection of all pins of instances that are
	//!        directly child of this module.
	Range<CollectionOfPinsFilteredByDirection>
	allInternalPins(const Direction direction) const;

	//! @brief Returns an iterable collection of all pins of instances that are
	//!        directly child of this module.
	Range<CollectionOfPins>
	allInternalPins() const;

	//! @brief Returns an iterable collection of all arcs of instances that are
	//!        directly child of this module.
	Range<CollectionOfArcs>
	allInternalArcs() const;

	//! @brief Returns an iterable collection of all pins instantiated in this
	//!        module in topological order (from inputs to outputs).
	std::vector<TupleElement<1, TopologicalIndex, Pin>>
	allPinsInTopologicalOrder() const;

	//! @brief Returns an iterable collection of all pins instantiated in this
	//!        module in reverse topological order (from outputs to inputs).
	std::vector<TupleElement<1, TopologicalIndex, Pin>>
	allPinsInReverseTopologicalOrder() const;

	//! @brief Returns an iterable collection of all nets instantiated in this
	//!        module in topological order (from inputs to outputs).
	//! @note  See Net::getTopologicalIndex() description to check how
	//!        the topological index of nets is defined.
	std::vector<TupleElement<1, TopologicalIndex, Net>>
	allNetsInTopologicalOrder() const;

	//! @brief Returns an iterable collection of all nets instantiated in this
	//!        module in reverse topological order (from outputs to inputs).
	//! @note  See Net::getTopologicalIndex() description to check how
	//!        the topological index of nets is defined.
	std::vector<TupleElement<1, TopologicalIndex, Net>>
	allNetsInReverseTopologicalOrder() const;

	//! @brief Returns an iterable collection of all instances instantiated in
	//!        this module in topological order (from inputs to outputs).
	//! @note  See Instance::getTopologicalIndex() description to check how
	//!        the topological index of instances is defined.
	std::vector<TupleElement<1, TopologicalIndex, Instance>>
	allInstancesInTopologicalOrder() const;

	//! @brief Returns a vector with the nets in the fanout cone of a pin. The
	//!        nets are sorted in breadth-first order (which is not the same as
	//!        topological order). Breadth-first order is typically faster than
	//!        topological order. If not null, the first net is always the net
	//!        to which the seed pins is connected to.
	std::vector<Rsyn::Net>
	getFanoutConeNetsInBreadthFirstOrder(Rsyn::Pin seed) const;

	//! @brief Returns a vector with the nets in the fan-in cone of a pin. The 
	//!        nets are sorted in breadth-first order (which is not the same as
	//!        topological order). Breadth-first order is typically faster than
	//!        topological order. If not null, the first net is always the net
	//!        to which the seed pins is connected to.
	std::vector<Rsyn::Net>
	getFaninConeNetsInBreadthFirstOrder(Rsyn::Pin seed) const;
	
}; // end class
	
} // end namespace