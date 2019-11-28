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

inline
Design 
Module::getDesign() {
	return data->moduleData->design;
} // end method

// -----------------------------------------------------------------------------

inline
const Design 
Module::getDesign() const {
	return data->moduleData->design;
} // end method

// -----------------------------------------------------------------------------

inline
int 
Module::generateNextSign() const {
	return ++data->moduleData->sign; /*must be pre-increment*/ 
} // end method

// -----------------------------------------------------------------------------

inline
int 
Module::getSign() const { 
	return data->moduleData->sign; 
} // end method

// -----------------------------------------------------------------------------

inline
Cell 
Module::createCell(const std::string &libraryCellName, const std::string &name) {
	Design design = getDesign(); 

	// TODO: move this test to design...
	LibraryCell lcell = design.findLibraryCellByName(libraryCellName);
	if (!lcell) {
		throw LibraryCellNotFoundException(libraryCellName);
	} // end if
	
	return design.createCell(*this, lcell, name);
} // end method

// -----------------------------------------------------------------------------

inline
Cell 
Module::createCell(const LibraryCell lcell, const std::string &name) {
	Design design = getDesign(); 
	return design.createCell(*this, lcell, name);
} // end method

// -----------------------------------------------------------------------------

inline
Port 
Module::createPort(const Direction &direction, const std::string &name) {
	Design design = getDesign(); 
	return design.createPort(*this, direction, name);	
} // end method

// -----------------------------------------------------------------------------

inline
Net 
Module::createNet(const std::string &name) {
	Design design = getDesign(); 
	
	// Check net name.
	if (design.findNetByName(name)) {
		throw NetAlreadyExistsException(name);
	} // end of

	// Creates and returns a new net;
	return design.createNet(*this, name);	
} // end method

// -----------------------------------------------------------------------------

inline
Port 
Module::getPortByIndex(const int index) {
	return data->moduleData->ports.get(index)->value; // TODO: awful
} // end method

// -----------------------------------------------------------------------------

inline
void
Module::getInstancesPerLogicalDepth(std::vector<std::vector<Rsyn::Instance>> &levels) {
	// TODO: For now let's just do a simple method to levelize. When more
	// unit tests are create, we should revise this method to see if a more
	// efficient one can be devised.
	
	levels.clear();	
	
	Rsyn::Attribute<Rsyn::Instance, int> depth = getDesign().createAttribute(-1);
	for (Rsyn::Instance instance : allInstancesInTopologicalOrder()) {
		// Set the depth as the largest depth of the driver cells (if any) plus
		// one.
		int lower = -1;
		
		switch (instance.getType()) {
			case Rsyn::CELL: {
				// We check the from pin of arcs instead of just checking the
				// input pins to handle the D pin of registers. We want that
				// the depth of a register is based on the depth of its CK pin.

				bool hasArcs = false;
				for (Rsyn::Arc arc : instance.allArcs()) {
					Rsyn::Net net = arc.getFromNet();
					if (net) {
						for (Rsyn::Pin driver : net.allPins(Rsyn::DRIVER)) {
							lower = std::max(lower, depth[driver.getInstance()]);
						} // end for
					} // end if
					hasArcs = true;
				} // end for
				
				// If no arcs, as a fall back, get the depth from the input
				// pins.
				if (!hasArcs) {
					for (Rsyn::Pin pin : instance.allPins()) {
						Rsyn::Net net = pin.getNet();
						if (net) {
							for (Rsyn::Pin driver : net.allPins(Rsyn::DRIVER)) {
								lower = std::max(lower, depth[driver.getInstance()]);
							} // end for
						} // end if
					} // end for					
				} // end if
				
				break;
			} // end case
			
			case Rsyn::MODULE: {
				for (Rsyn::Pin pin : instance.allPins()) {
					Rsyn::Net net = pin.getNet();
					if (net) {
						for (Rsyn::Pin driver : net.allPins(Rsyn::DRIVER)) {
							lower = std::max(lower, depth[driver.getInstance()]);
						} // end for
					} // end if
				} // end for	
				
				break;
			} // end case
			
			case Rsyn::PORT: {
				Rsyn::Port port = instance.asPort();
				switch (port.getDirection()) {
					case Rsyn::IN: {
						// Nothing to be done here... (depth = 0)
						break;
					} // end case
					
					case Rsyn::OUT: {
						Rsyn::Net net = port.getInnerPin().getNet();
						if (net) {
							for (Rsyn::Pin driver : net.allPins(Rsyn::DRIVER)) {
								lower = std::max(lower, depth[driver.getInstance()]);
							} // end for
						} // end if						
						break;
					} // end case
					
					default:
						throw Exception("Port direction not supported.");
				} // end switch
			
				break;
			} // end case
			
			default:
				throw Exception("unsupported type");
		} // end switch
		
		// Increment lower to get the instance depth.
		lower += 1;
		
		// Stores the instance depth.
		depth[instance] = lower;
		
		// Add instance to the level vector.
		if (levels.size() <= lower) {
			levels.resize(lower + 1);
		} // end if
		levels[lower].push_back(instance);
	} //  end for
} // end method

// -----------------------------------------------------------------------------

inline
void
Module::getNetsPerLogicalDepth(std::vector<std::vector<Rsyn::Net>> &levels) {
	levels.clear();
	std::cout << "TODO: getNetsPerLogicalDepth() not implemented\n";
} // end method

// -----------------------------------------------------------------------------

inline
int
Module::getNumPorts(const Direction direction) const {
	return data->moduleData->portsByDirection[direction].size();
} // end method

// -----------------------------------------------------------------------------

inline
Range<ReferenceListCollection<Instance>>
Module::allInstances() const {
	return ReferenceListCollection<Instance>(data->moduleData->instances);
} // end method

// -----------------------------------------------------------------------------

inline
Range<ReferenceListCollection<Port>>
Module::allPorts() const {
	return ReferenceListCollection<Port>(data->moduleData->ports);
} // end method

// -----------------------------------------------------------------------------

inline
std::set<Port> &
Module::allPorts(const Rsyn::Direction direction) const {
	return data->moduleData->portsByDirection[direction];
} // end method

// -----------------------------------------------------------------------------

inline
Range<ReferenceListCollection<Net>>
Module::allNets() const {
	return ReferenceListCollection<Net>(data->moduleData->nets);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPinsFilteredByDirection>
Module::allInterfacePins(const Direction direction) const {
	return Instance::allPins(direction);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPins>
Module::allInterfacePins() const {
	return Instance::allPins();
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfArcs>
Module::allInterfaceArcs() const {
	return Instance::allArcs();
} // end method

// -----------------------------------------------------------------------------

inline
std::vector<TupleElement<1, TopologicalIndex, Pin>>
Module::allPinsInTopologicalOrder() const {
	const int numNets = data->moduleData->nets.size();

	std::vector<TupleElement<1, TopologicalIndex, Pin>> sortedPins;
	sortedPins.reserve(numNets*3);

	for (Rsyn::Instance instance : allInstances()) {
		for (Rsyn::Pin pin : instance.allPins())
			sortedPins.push_back(std::make_tuple(pin.getTopologicalIndex(), pin));
	} // end for

	std::sort(sortedPins.begin(), sortedPins.end());
	return sortedPins;
} // end method

// -----------------------------------------------------------------------------

inline
std::vector<TupleElement<1, TopologicalIndex, Pin>>
Module::allPinsInReverseTopologicalOrder() const {
	const int numNets = data->moduleData->nets.size();

	std::vector<TupleElement<1, TopologicalIndex, Pin>> sortedPins;
	sortedPins.reserve(numNets*3);

	for (Rsyn::Instance instance : allInstances()) {
		for (Rsyn::Pin pin : instance.allPins())
			sortedPins.push_back(std::make_tuple(-pin.getTopologicalIndex(), pin));
	} // end for

	std::sort(sortedPins.begin(), sortedPins.end());
	return sortedPins;
} // end method
// -----------------------------------------------------------------------------

inline
std::vector<TupleElement<1, TopologicalIndex, Net>>
Module::allNetsInTopologicalOrder() const {
	const int numNets = data->moduleData->nets.size();
	
	std::vector<TupleElement<1, TopologicalIndex, Net>> sortedNets;
	sortedNets.resize(numNets);
	
	int i = 0;
	for (Rsyn::Net net : allNets()) {
		sortedNets[i++] = std::make_tuple(net.getTopologicalIndex(), net);
	} // end for
	sortedNets.resize(i);
	std::sort(sortedNets.begin(), sortedNets.end());
		
	return sortedNets;
} // end method

// -----------------------------------------------------------------------------

inline
std::vector<TupleElement<1, TopologicalIndex, Net>>
Module::allNetsInReverseTopologicalOrder() const {
	const int numNets = data->moduleData->nets.size();
	
	std::vector<TupleElement<1, TopologicalIndex, Net>> sortedNets;
	sortedNets.resize(numNets);
	
	int i = 0;
	for (Rsyn::Net net : allNets()) {
		sortedNets[i++] = std::make_tuple(-net.getTopologicalIndex(), net);
	} // end for
	sortedNets.resize(i);
	std::sort(sortedNets.begin(), sortedNets.end());
		
	return sortedNets;
} // end method

// -----------------------------------------------------------------------------

inline
std::vector<TupleElement<1, TopologicalIndex, Instance>>
Module::allInstancesInTopologicalOrder() const {
	const int numInstances = data->moduleData->instances.size();
	
	std::vector<TupleElement<1, TopologicalIndex, Instance>> sortedInstances;
	sortedInstances.resize(numInstances);
	
	int i = 0;
	for (Rsyn::Instance instance : allInstances()) {
		sortedInstances[i++] = std::make_tuple(instance.getTopologicalIndex(), instance);
	} // end for
	sortedInstances.resize(i);
	std::sort(sortedInstances.begin(), sortedInstances.end());
		
	return sortedInstances;
} // end method

////////////////////////////////////////////////////////////////////////////////
// Sorted Objects
////////////////////////////////////////////////////////////////////////////////	

inline
std::vector<Rsyn::Net> 
Module::getFanoutConeNetsInBreadthFirstOrder(Rsyn::Pin seed) const {
	std::vector<Rsyn::Net> result;	
	std::queue<Rsyn::Net> open;
	
	// Push seed net.
	Rsyn::Net net = seed.getNet();
	if (net)
		open.push(net);
	
	// Breadth-first search.
	const int sign = generateNextSign();
	
	while (!open.empty()) {
		Rsyn::Net currentNet = open.front();
		open.pop();
		
		if (currentNet->sign == sign)
			continue;
		
		currentNet->sign = sign;
		result.push_back(currentNet);
		
		// Add neighbors.
		for (Rsyn::Pin sink : currentNet.allPins(SINK)) {
			for (Rsyn::Arc arc : sink.allOutgoingArcs()) {
				Rsyn::Net net = arc.getToNet();
				if (net && net->sign != sign)
					open.push(net);				
			} // end for
		} // end for
	} // end while
	
	// Return.
	return result;
} // end method

// -----------------------------------------------------------------------------

inline
std::vector<Rsyn::Net>
Module::getFaninConeNetsInBreadthFirstOrder(Rsyn::Pin seed) const {
	std::vector<Rsyn::Net> result;	
	std::queue<Rsyn::Net> open;
	
	// Push seed net.
	Rsyn::Net net = seed.getNet();
	if (net)
		open.push(net);
	
	// Breadth-first search.
	const int sign = generateNextSign();
	
	while (!open.empty()) {
		Rsyn::Net currentNet = open.front();
		open.pop();
		
		if (currentNet->sign == sign)
			continue;
		
		currentNet->sign = sign;
		result.push_back(currentNet);
		
		// Add neighbors.
		for (Rsyn::Pin driver : currentNet.allPins(DRIVER)) {
			for (Rsyn::Arc arc : driver.allIncomingArcs()) {
				Rsyn::Net net = arc.getFromNet();
				if (net && net->sign != sign)
					open.push(net);		
			} // end for
		} // end for
	} // end while
	
	// Return.
	return result;
} // end method

} // end namespace