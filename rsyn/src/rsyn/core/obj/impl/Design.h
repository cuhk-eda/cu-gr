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
Design::Design(const std::string &name) {
	create(name);
} // end constructor

// -----------------------------------------------------------------------------

inline
void
Design::create(const std::string &name) {
	// Allocate memory for the new design.
	//data = std::make_shared<DesignData>();
	data = new DesignData;
	data->name = name;
	
	// If no top is specified, just create an empty top for the design.
	data->topModule = createModule(nullptr, "__root");
} // end method

// -----------------------------------------------------------------------------

inline
Module 
Design::getTopModule() {
	return data->topModule;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
Design::getName() const {
	return data->name;
} // end method

// -----------------------------------------------------------------------------

inline 
void 
Design::updateName(std::string name) {
	data->name = name;
} // end method

// -----------------------------------------------------------------------------

inline
std::string 
Design::generateUniqueInstanceName(const std::string &prefix) {
	std::ostringstream oss;
	oss << prefix << data->anonymousInstanceId; 
	data->anonymousInstanceId++;	
	return oss.str();
} // end method

// -----------------------------------------------------------------------------

inline
std::string 
Design::generateUniqueNetName(const std::string &prefix) {
	std::ostringstream oss;
	oss << prefix << data->anonymousNetId; 
	data->anonymousNetId++;	
	return oss.str();
} // end method

// -----------------------------------------------------------------------------

inline
int 
Design::generateNextSign() { 
	return ++data->sign; /*must be pre-increment*/ 
} // end method

// -----------------------------------------------------------------------------

inline
int 
Design::getSign() const { 
	return data->sign; 
} // end method

// -----------------------------------------------------------------------------

inline 
Instance 
Design::findInstanceByName(const std::string &name) const {
	auto it = data->instanceMapping.find(name);
	return it != data->instanceMapping.end()? it->second : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline 
Cell 
Design::findCellByName(const std::string &name) const {
	Rsyn::Instance instance = findInstanceByName(name);
	if (instance) {
		return instance.getType() == Rsyn::CELL? instance.asCell() : nullptr;
	} else {
		return nullptr;
	} // end else
} // end method

// -----------------------------------------------------------------------------

inline 
Port 
Design::findPortByName(const std::string &name) const {
	Rsyn::Instance instance = findInstanceByName(name);
	if (instance) {
		return instance.getType() == Rsyn::PORT? instance.asPort() : nullptr;
	} else {
		return nullptr;
	} // end else
} // end method

// -----------------------------------------------------------------------------

inline 
Module 
Design::findModuleByName(const std::string &name) const {
	Rsyn::Instance instance = findInstanceByName(name);
	if (instance) {
		return instance.getType() == Rsyn::MODULE? instance.asModule() : nullptr;
	} else {
		return nullptr;
	} // end else
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell 
Design::findLibraryCellByName(const std::string &name) const {
	auto it = data->libraryCellMapping.find(name);
	return it != data->libraryCellMapping.end()? it->second : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Net 
Design::findNetByName(const std::string &name) const {
	auto it = data->netMapping.find(name);
	return it != data->netMapping.end()? it->second : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline 
Pin 
Design::findPinByName(const std::string &cellName, const std::string &pinName) const {
	Cell cell = findCellByName(cellName);
	if (cell) {
		return cell.getPinByName(pinName);
	} else {
		return nullptr;
	} // end else
} // end method

// -----------------------------------------------------------------------------

inline 
Pin 
Design::findPinByName(const std::string &name, const std::string::value_type separator) const {
	std::size_t split = name.find_first_of(separator); 
	if (split == std::string::npos)
		return nullptr;

	const std::string cellName = name.substr(0, split);
	const std::string pinName = name.substr(split + 1, std::string::npos);
	return findPinByName(cellName, pinName);
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell 
Design::createLibraryCell(const CellDescriptor &dscp, const bool ignoreDuplicated) {
	if (findLibraryCellByName(dscp.getName())) {
		if (!ignoreDuplicated) {
			throw LibraryCellAlreadyExistsException(dscp.getName());
		} else {
			return nullptr;
		} // end if
	} // end if

	const int numPins = (int) dscp.pins.size();
	const int numArcs = (int) dscp.arcs.size();

	// Check if it's valid descriptor.
	dscp.checkConsistency();
	
	// Creates a new cell in the data structure.
	LibraryCellData * lcell = &(data->libraryCells.create()->value); // TODO: awful
	LibraryCell libraryCell(lcell);

	// Initializes cell.
	lcell->id = data->libraryCells.lastId();
	lcell->design = *this;
	lcell->name = dscp.getName();

	// Stores library cell name.
	data->libraryCellMapping[lcell->name] = libraryCell;	

	// Sort pin by names to ensure same indexes among different cells with
	// the same footprint.
	std::map<std::string, int> mapping;

	for (int i = 0; i < numPins; i++) {
		const std::tuple<std::string, Direction, Use> &t = dscp.pins[i];
		mapping[std::get<0>(t)] = i;
	} // end for	

	// Initializes library cell's pins.
	lcell->pins.resize(numPins);
	for (auto element : mapping) {
		const int index = element.second;
		const std::tuple<std::string, Direction, Use> &t = dscp.pins[index];

		LibraryPinData * lpin = &(data->libraryPins.create()->value); // TODO: awful
		lpin->id = data->libraryPins.lastId();
		lpin->design = *this;
		lpin->index = index;
		lpin->lcell = libraryCell;
		lpin->name = std::get<0>(t);
		lpin->direction = std::get<1>(t);
		// Mateus @ 20180917: Adding PinUse
		lpin->pinUse = std::get<2>(t);

		lcell->pins[index] = LibraryPin(lpin);
		switch (std::get<1>(t)){
			case IN :
				lcell->numInputPins++;
				break;
			case OUT:
				lcell->numOutputPins++;
				break;
			case BIDIRECTIONAL: 
				lcell->numInOutPins++;
				break;
		} // end switch 
	} // end for

	// Initializes library cell's pins.
	lcell->arcs.resize(numArcs);
	for (int i = 0; i < numArcs; i++) {	
		LibraryArcData * larc = &(data->libraryArcs.create()->value); // TODO: awful
		larc->id = data->libraryArcs.lastId();
		larc->design = *this;
		larc->lcell = libraryCell;
		larc->index = i;
		larc->from = libraryCell.getLibraryPinByName(std::get<0>(dscp.arcs[i]));
		larc->to = libraryCell.getLibraryPinByName(std::get<1>(dscp.arcs[i]));
		
		lcell->arcs[i] = larc;
	} // end for
	
	return libraryCell;
} // end method

// -----------------------------------------------------------------------------

inline
Cell 
Design::createCell(const Module parent, const LibraryCell libraryCell, const std::string &name) {
	const LibraryCellData * lcell = libraryCell.data;
	
	const std::string &cellName = name == ""? 
		generateUniqueInstanceName("__cell") : name;
	
	const int numPins = lcell->pins.size();
	const int numArcs = lcell->arcs.size();
	
	// Creates a new cell in the data structure.
	InstanceData * instance = &(data->instances.create()->value); // TODO: awful
	Cell cell(instance);
	
	// Initializes instance.
	instance->id = data->instances.lastId();
	instance->design = *this;
	instance->parent = parent;
	instance->lcell = const_cast<LibraryCellData *>(lcell);
	instance->type = CELL;
	instance->pins.resize(numPins);
	instance->arcs.resize(numArcs);

	// Initializes instance's pins.
	for (int i = 0; i < numPins; i++) {
		PinData * pin = &(data->pins.create()->value); // TODO: awful
		LibraryPin lpin = lcell->pins[i];
		pin->id = data->pins.lastId();
		pin->instance = cell;
		pin->direction = lpin.data->direction;
		pin->type = Rsyn::CELL;
		pin->index = lpin.data->index;
		
		instance->pins[i] = pin;
		
		switch (pin->direction) {
			case Rsyn::IN:
				pin->order = 0;
				break;
			case Rsyn::OUT:
				pin->order = TOPOLOGICAL_SORTING_SMALL_GAP;
				break;
			case Rsyn::BIDIRECTIONAL:
				throw Exception("Rsyn does not support bidirectional pins yet.\n");
			default:
				throw Exception("Unknown pin direction.\n");
		} // end switch
	} // end for
	
	// Initializes cell's arcs.
	for (int i = 0; i < numArcs; i++) {
		ArcData * arc = &(data->arcs.create()->value); // TODO: awful
		LibraryArc larc = lcell->arcs[i];
		arc->id = data->arcs.lastId();
		arc->type = INSTANCE_ARC;
		arc->libraryArcData = larc.data;
		arc->from = cell.getPinByLibraryPin(larc.data->from);
		arc->to = cell.getPinByLibraryPin(larc.data->to);
			
		arc->from.data->arcs[FORWARD].push_back(Arc(arc));
		arc->to.data->arcs[BACKWARD].push_back(Arc(arc));
		
		instance->arcs[i] = arc;
	} // end for
			
	// Stores cell name.
	if (data->instanceNames.size() <= instance->id) {
		data->instanceNames.resize(instance->id+1);
	} // end if

	data->instanceNames[instance->id] = cellName;
	data->instanceMapping[cellName] = Instance(instance);

	// Records this cell in it's parent module.
	parent->moduleData->instances.add(cell);
	cell->mid = parent->moduleData->instances.lastId();
	
	// Trace the number of instances.
	data->instanceCount[Rsyn::CELL]++;
	
	// Mark as dirty.
	data->dirty = true;
	
	// Notify observers.
	for (auto f : data->observers[EVENT_POST_INSTANCE_CREATE])
		f->onPostInstanceCreate(cell);
	
	// Return
	return cell;
} // end method

// -----------------------------------------------------------------------------

inline
Port 
Design::createPort(Module parent, const Direction direction, const std::string &name) {
	const std::string &portName = name == ""? 
		generateUniqueInstanceName("__cell") : name;
	
	const int numPins = 1;
	const int numArcs = 0;
	
	// Creates a new cell in the data structure.
	Port port = &(data->instances.create()->value); // TODO: awful

	// Creates the outer pin which will be added to the parent module.
	PinData * outer = &(data->pins.create()->value); // TODO: awful
	outer->id = data->pins.lastId();
	outer->instance = parent;
	outer->direction = direction;
	outer->type = Rsyn::MODULE;
	outer->boundary = true;
	outer->index = parent->pins.size();
	parent->pins.push_back(outer);
	
	// Initializes port data.
	port->id = data->instances.lastId();
	port->design = *this;
	port->parent = parent;
	port->type = PORT;
	port->outerPin = outer;
	port->lcell = nullptr;
	port->pins.resize(numPins);
	port->arcs.resize(numArcs);

	// Creates the inner pin which will be added to the port.
	PinData * inner = &(data->pins.create()->value); // TODO: awful
	inner->id = data->pins.lastId();
	inner->instance = port;
	inner->direction = Global::getReverseDirection(direction);
	inner->type = Rsyn::PORT;
	inner->boundary = true;
	inner->index = 0;
	port->pins[0] = inner;	

	// Topological index.
	switch (direction) {
		case Rsyn::IN:
			outer->order = 0;
			inner->order = TOPOLOGICAL_SORTING_SMALL_GAP;
			break;
		case Rsyn::OUT:
			inner->order = 1;
			outer->order = 2;
			break;
		default:
			throw Exception("Port direction not supported.");
	} // ens switch
	
	// Stores port (instance) name.
	if (data->instanceNames.size() <= port->id) {
		data->instanceNames.resize(port->id+1);
	} // end if

	data->instanceNames[port->id] = portName;
	data->instanceMapping[portName] = Instance(port);

	// Records this cell in it's parent module.
	parent->moduleData->instances.add(port);
	port->mid = parent->moduleData->instances.lastId();
	parent->moduleData->ports.add(port);
	parent->moduleData->portsByDirection[direction].insert(port);
	
	// Trace the number of instances.
	data->instanceCount[Rsyn::PORT]++;	
	
	// Mark as dirty.
	data->dirty = true;	
	
	// Notify observers.
	for (auto f : data->observers[EVENT_POST_INSTANCE_CREATE])
		f->onPostInstanceCreate(port);

	// Return
	return port;
} // end method

// -----------------------------------------------------------------------------

inline
Module 
Design::createModule(const LibraryModule lmoudle, const std::string &name) {
	// Creates a new cell in the data structure.
	InstanceData * instance = &(data->instances.create()->value); // TODO: awful
	
	// Initializes cell.
	instance->id = data->instances.lastId();
	instance->type = Rsyn::MODULE;
	instance->design = *this;
	instance->lcell = nullptr;
	instance->moduleData = new ModuleData;
	instance->moduleData->design = *this;
				
	// Stores instance name.
	if (data->instanceNames.size() <= instance->id) {
		data->instanceNames.resize(instance->id+1);
	} // end if

	data->instanceNames[instance->id] = name;
	data->instanceMapping[name] = Instance(instance);
	
	// Trace the number of instances.
	data->instanceCount[Rsyn::MODULE]++;		
	
	// Mark as dirty.
	data->dirty = true;
	
	// Notify observers.
	for (auto f : data->observers[EVENT_POST_INSTANCE_CREATE])
		f->onPostInstanceCreate(instance);
	
	// Return
	return Module(instance);
} // end method

// -----------------------------------------------------------------------------

inline
Net 
Design::createNet(const Module parent, const std::string &name) {
	// Creates a new net in the data structure.
	NetData * net = &(data->nets.create()->value); // TODO: awful
	
	// Gets the net name.
	const std::string &netName = name == ""? 
		generateUniqueNetName("__net") : name;	
	
	// Initializes net.
	net->id = data->nets.lastId();
	net->parent = parent;

	// Stores net name.
	if (data->netNames.size() <= net->id) {
		data->netNames.resize(net->id+1);
	} // end if

	data->netNames[net->id] = netName;
	data->netMapping[netName] = net;
		
	// Records this cell in it's parent module.
	parent->moduleData->nets.add(net);
	net->mid = parent->moduleData->nets.lastId();
	
	// Notify observers.
	for (auto f : data->observers[EVENT_POST_NET_CREATE])
		f->onPostNetCreate(net);
	
	// Mark as dirty.
	data->dirty = true;	
	
	// Return.
	return net;
} // end method

// -----------------------------------------------------------------------------

inline
void 
Design::connectPin(Pin pin, Net net) {
	if (pin->net) {
		disconnectPin(pin);
	} // end if
	
	net->pins.push_back(pin);
	pin->net = net;
	
	net->numPinsOfType[pin.getDirection()]++;
	if (pin.getDirection() == DRIVER) {
		net->driver = pin; // cache the driver
	} // end if

	// Create net arcs as necessary.
	switch (pin.getDirection()) {
		case Rsyn::DRIVER: {
			for (Rsyn::Pin sink : net.allPins(Rsyn::SINK)) {
				Arc arc = &(data->arcs.create()->value); // TODO: awful
				arc->id = data->arcs.lastId();
				arc->type = NET_ARC;
				arc->netData = net.data;
				arc->from = pin;
				arc->to = sink;

				pin->arcs[FORWARD].push_back(arc);
				sink->arcs[BACKWARD].push_back(arc);
			} // end for	
			break;
		} // end case
		
		case Rsyn::SINK: {
			for (Rsyn::Pin driver : net.allPins(Rsyn::DRIVER)) {
				Arc arc = &(data->arcs.create()->value); // TODO: awful
				arc->id = data->arcs.lastId();
				arc->type = NET_ARC;
				arc->netData = net.data;
				arc->from = driver;
				arc->to = pin;

				driver->arcs[FORWARD].push_back(arc);
				pin->arcs[BACKWARD].push_back(arc);
			} // end for
			break;
		} // end case
		
		default:
			throw Exception("Unsupported direction.");
	} // end switch
	
	// Mark as dirty.
	data->dirty = true;	
		
	// Update topological sorting.
	updateTopologicalIndex(pin);
	
	// Notify observers.
	for (auto f : data->observers[EVENT_POST_PIN_CONNECT])
		f->onPostPinConnect(pin);
} // end method

// -----------------------------------------------------------------------------

inline
void
Design::disconnectPin(Pin pin) {
	// Notify observers.
	for (auto f : data->observers[EVENT_PRE_PIN_DISCONNECT])
		f->onPrePinDisconnect(pin);
	
	if (pin->net) {
		// Remove the pin from the net.
		Net net = pin->net;
		const int numPins = net->pins.size();
		for (int i = 0; i < numPins; i++) {
			if (net->pins[i] == pin) {
				// Swap the element to be delete with the last one and then shrink
				// the vector. Note that is affect the pin ordering inside the net.			
				std::swap(net->pins[i], net->pins.back());
				net->pins.resize(net->pins.size() - 1);
				break;
			} // end if
		} // end for
		
		// Remove arcs from the pin.
		switch (pin.getDirection()) {
			case Rsyn::DRIVER:
				pin->arcs[FORWARD].clear();
				break;
			case Rsyn::SINK:
				pin->arcs[BACKWARD].clear();
				break;
			default:
				throw Exception("Unsupported direction.");
		} // end switch

		// Disconnect the pin.
		pin->net = nullptr;

		// Adjust pins count.
		net->numPinsOfType[pin.getDirection()]--;
		
		// Update the cached driver if necessary.
		if (pin.getDirection() == Rsyn::DRIVER) {
			for (Pin pin : net.allPins(Rsyn::OUT)) {
				net->driver = pin;
				break;
			} // end for			
		} // end if

		// Mark as dirty.
		data->dirty = true;
	} // end if
} // end method

// -----------------------------------------------------------------------------

// TODO: Improve performance of remaping. The first thing to do is to cache
// the remaping table.

inline
void 
Design::remap(Cell cell, LibraryCell newLibraryCell) {
	LibraryCell oldLibraryCell = cell.getLibraryCell();
	
	// Check if the cells have the same number of arcs...
	// TODO: a deep comparison is necessary.
	if (oldLibraryCell.getNumArcs() != newLibraryCell.getNumArcs()) {
		throw IncompatibleLibraryCellForRemapping(
				oldLibraryCell.getName(), newLibraryCell.getName());
	} // end if
	
	// Check if the cells have the same number of pins...
	if (oldLibraryCell.getNumPins() != newLibraryCell.getNumPins()) {
		throw IncompatibleLibraryCellForRemapping(
				oldLibraryCell.getName(), newLibraryCell.getName());
	} // end if	
	
	// Create a mapping between the old and new pins.
	// TODO: Cache this!
	std::unordered_map<Rsyn::LibraryPin, Rsyn::LibraryPin> mapLibraryPins;
	for (LibraryPin oldLibraryPin : oldLibraryCell.allLibraryPins()) {
		LibraryPin newLibraryPin = 
				newLibraryCell.getLibraryPinByName(oldLibraryPin.getName());
		
		if (!newLibraryPin) {
			throw IncompatibleLibraryCellForRemapping(
					oldLibraryCell.getName(), newLibraryCell.getName());
		} // end if
		
		mapLibraryPins[oldLibraryPin] = newLibraryPin;
	} // end method

	// Create a mapping between the old and new arcs.
	// TODO: Cache this!
	std::unordered_map<Rsyn::LibraryArc, Rsyn::LibraryArc> mapLibraryArcs;
	for (LibraryArc oldLibraryArc : oldLibraryCell.allLibraryArcs()) {
		LibraryArc newLibraryArc =
				newLibraryCell.getLibraryArcByPinNames(oldLibraryArc.getFromName(), oldLibraryArc.getToName());

		if (!newLibraryArc) {
			throw IncompatibleLibraryCellForRemapping(
					oldLibraryCell.getName(), newLibraryCell.getName());
		} // end if

		mapLibraryArcs[oldLibraryArc] = newLibraryArc;
	} // end method

	// Update library cell.
	cell->lcell = newLibraryCell.data;

	// Update pins.
	for (Pin pin : cell.allPins()) {
		Rsyn::LibraryPin newLibraryPin = mapLibraryPins[oldLibraryCell.getLibraryPinByIndex(pin->index)];
		pin->index = newLibraryPin->index;
	} // end for

	// Update arcs.
	for (Arc arc : cell.allArcs()) {
		Rsyn::LibraryArc newLibraryArc = mapLibraryArcs[arc.getLibraryArc()];
		arc->libraryArcData = newLibraryArc.data;
	} // end for
	
	// Notify observers.
	for (auto f : data->observers[EVENT_POST_CELL_REMAP])
		f->onPostCellRemap(cell, oldLibraryCell);
} // end method

// -----------------------------------------------------------------------------

inline
void 
Design::remap(Cell cell, const std::string &newLibraryCellName) {
	LibraryCell newLibraryCell = findLibraryCellByName(newLibraryCellName);
	if (!newLibraryCell) {
		throw LibraryCellNotFoundException(newLibraryCellName);
	} // end if
	remap(cell, newLibraryCell);
} // end method

// -----------------------------------------------------------------------------

inline
int
Design::getNumInstances() const {
	return data->instances.size();
} // end method

// -----------------------------------------------------------------------------

inline
Range<ReferenceListCollection<Cell>>
Design::getAllCells() const {
    for (Instance inst : data->topModule.allInstances()){
        if (inst.getType() != CELL)
            continue;
        data->cells.add(inst.asCell());
    }
    Range<ReferenceListCollection<Cell>> a = ReferenceListCollection<Cell>(data->cells);
    for (Cell c : a){
        std::cout << c.getName() << "\n";
        break;
    }
        return ReferenceListCollection<Cell>(data->cells); 
} // end method

// -----------------------------------------------------------------------------

inline
int
Design::getNumInstances(const InstanceType type) const {
	return data->instanceCount[type];
} // end method

// -----------------------------------------------------------------------------

inline
int
Design::getNumNets() const {
	return data->nets.size();
} // end method

// -----------------------------------------------------------------------------

inline
int
Design::getNumPins() const {
	return data->pins.size();
} // end method

////////////////////////////////////////////////////////////////////////////////
// Topological Ordering
////////////////////////////////////////////////////////////////////////////////

inline
void
Design::updateTopologicalIndex(Pin pin) {
	// Some checks...
	static_assert(std::is_integral<TopologicalIndex>::value, "Integer required.");
	static_assert(std::is_signed<TopologicalIndex>::value, "Signed type required.");
	
	static_assert(TOPOLOGICAL_SORTING_SMALL_GAP > 0, "small < 0");
	static_assert(TOPOLOGICAL_SORTING_LARGE_GAP > 0, "large < 0");
	static_assert(TOPOLOGICAL_SORTING_SMALL_GAP <= 
			TOPOLOGICAL_SORTING_LARGE_GAP, "small gap > large gap");
	
	// Gets the signature for this update search.
	const int sign = generateNextSign();
	
	// Gets the lower bound index.
	TopologicalIndex lower = 
			-std::numeric_limits<TopologicalIndex>::infinity();
	bool hasLower = false;
	for (Rsyn::Pin predecessor : pin.allPredecessorPins(true)) {
		lower = std::max(lower, predecessor.getTopologicalIndex());
		hasLower = true;
	} // end for
	
	// Gets the upper bound index.
	TopologicalIndex upper = 			
			+std::numeric_limits<TopologicalIndex>::infinity();
	bool hasUpper = false;
	for (Rsyn::Pin successor : pin.allSucessorPins(true)) {
		upper = std::min(upper, successor.getTopologicalIndex());
		hasUpper = true;
	} // end for
	
	// Set pin's topological ordering.
	if (!hasLower && !hasUpper) {
		pin->order = 0;
	} else if (hasLower && !hasUpper) {
		pin->order = lower + TOPOLOGICAL_SORTING_SMALL_GAP;
	} else if (!hasLower && hasUpper) {
		pin->order = upper - TOPOLOGICAL_SORTING_SMALL_GAP;
	} else {
		// Ok, we got a lower and an upper bound...	
		
		if (lower < upper && (upper - lower >= 2)) {
			// The easy case...
			// Since we use integral types, we must guarantee that there's room
			// for the new pin between the lower and upper bounds, that's why
			// we check if (upper - lower >= 2).
			pin->order = (lower + upper) / 2;
		} else {
			// We need to propagate the indexes...
			const TopologicalIndex smallGap = TOPOLOGICAL_SORTING_SMALL_GAP;
			const TopologicalIndex largeGap = TOPOLOGICAL_SORTING_LARGE_GAP;
			
			const TopologicalIndex left0 = upper;
			const TopologicalIndex left1 = lower + smallGap + 1;
			const TopologicalIndex right = left1 + largeGap;
			const TopologicalIndex w0 = (right - left0);
			const TopologicalIndex w1 = (right - left1);
						
			pin->order = (lower + left1) / 2;
			
			std::queue<std::tuple<Rsyn::Pin, TopologicalIndex>> open;
			for (Rsyn::Pin successor : pin.allSucessorPins(true)) {
				if (successor->order < right) {
					open.push(std::make_tuple(successor, pin->order));
				} // end if
			} // end for			

			while (!open.empty()) {
				Rsyn::Pin current = std::get<0>(open.front());
				const TopologicalIndex generatorOrder = std::get<1>(open.front());
				open.pop();

				if (current == pin) {
					// loop detected;
					std::cout << "WARNING: Loop detected.\n";
					continue;
				} // end if
				
				if (current->order > generatorOrder) {
					// no need to continue propagating...
					continue;
				} // end if
					
				TopologicalIndex order;
				if (current->sign != sign) {
					current->sign = sign;
					order = (TopologicalIndex) std::floor(
							(float((current->order - upper)*w1)/float(w0)) + left1);
					if (order <= generatorOrder) {
						order = generatorOrder + smallGap;
					} // end if						
				} else {
					order = generatorOrder + smallGap;
				} // end else
				
				current->order = order;
				
				for (Rsyn::Pin successor : current.allSucessorPins(true)) {
					if (successor->order <= order) {
						open.push(std::make_tuple(successor, order));
					} // end if
				} // end for
			} // end while
		} // end else
	} // end else
} // end method

////////////////////////////////////////////////////////////////////////////////
// Unique Identifiers for Rsyn Objects
////////////////////////////////////////////////////////////////////////////////

inline Index Design::getId(Net net) const { return net->id; }
inline Index Design::getId(Instance instance) const { return instance->id; }
inline Index Design::getId(Pin pin) const { return pin->id; }
inline Index Design::getId(Arc arc) const { return arc->id; }
inline Index Design::getId(LibraryCell lcell) const { return lcell->id; }
inline Index Design::getId(LibraryPin lpin) const { return lpin->id; }
inline Index Design::getId(LibraryArc larc) const { return larc->id; }

////////////////////////////////////////////////////////////////////////////////
// Events
////////////////////////////////////////////////////////////////////////////////

template<class T>
inline
void
Design::registerObserver(T *observer) {
	static_assert(std::is_base_of<DesignObserver, T>::value,
			"Unable to register class as observer. "
			"The observer class must inherit from Rsyn::Observer.");

	observer->DesignObserver::observedDesign = Design(data);
	
	// Check if the observer implements (overwrites) the event callbacks if so
	// register it to receive notifications of the respective event.

	if (typeid(&DesignObserver::onDesignDestruction) != typeid(&T::onDesignDestruction)) {
		data->observers[EVENT_DESTRUCTION].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPostInstanceCreate) != typeid(&T::onPostInstanceCreate)) {
		data->observers[EVENT_POST_INSTANCE_CREATE].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPreInstanceRemove) != typeid(&T::onPreInstanceRemove)) {
		data->observers[EVENT_PRE_INSTANCE_REMOVE].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPostNetCreate) != typeid(&T::onPostNetCreate)) {
		data->observers[EVENT_POST_NET_CREATE].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPreNetRemove) != typeid(&T::onPreNetRemove)) {
		data->observers[EVENT_PRE_NET_REMOVE].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPostCellRemap) != typeid(&T::onPostCellRemap)) {
		data->observers[EVENT_POST_CELL_REMAP].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPostPinConnect) != typeid(&T::onPostPinConnect)) {
		data->observers[EVENT_POST_PIN_CONNECT].push_back(observer);
	} // end if

	if (typeid(&DesignObserver::onPrePinDisconnect) != typeid(&T::onPrePinDisconnect)) {
		data->observers[EVENT_PRE_PIN_DISCONNECT].push_back(observer);
	} // end if	

	if (typeid (&DesignObserver::onPostInstanceMove) != typeid (&T::onPostInstanceMove)) {
		data->observers[EVENT_POST_INSTANCE_MOVE].push_back(observer);
	} // end if

} // end method

// -----------------------------------------------------------------------------

inline
void
Design::unregisterObserver(DesignObserver *observer) {
	for (int i = 0; i < NUM_DESIGN_EVENTS; i++) {
		data->observers[i].remove(observer);
	} // end for
	observer->DesignObserver::observedDesign = nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
void
Design::notifyInstancePlaced(Rsyn::Instance instance, Rsyn::DesignObserver *ignoreObserver) {
	for (DesignObserver * observer : data->observers[EVENT_POST_INSTANCE_MOVE]) {
		if (observer != ignoreObserver) {
			observer->onPostInstanceMove(instance);
		} // end if
	} // end for
} // end method

////////////////////////////////////////////////////////////////////////////////
// Attributes
////////////////////////////////////////////////////////////////////////////////

inline 
AttributeInitializer 
Design::createAttribute() { 
	return AttributeInitializer(*this); 
} // end method

// -----------------------------------------------------------------------------
 
template<typename DefaultValueType>
inline
AttributeInitializerWithDefaultValue<DefaultValueType>
Design::createAttribute(const DefaultValueType &defaultValue) {
	return AttributeInitializerWithDefaultValue<DefaultValueType>(*this, defaultValue); 
} // end method

////////////////////////////////////////////////////////////////////////////////
// Tags
////////////////////////////////////////////////////////////////////////////////

inline
NetTag 
Design::getTag(Rsyn::Net net) {
	return NetTag(&net->tag);
} // end method

// -----------------------------------------------------------------------------

inline
InstanceTag
Design::getTag(Rsyn::Instance instance) {
	return InstanceTag(&instance->tag);
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCellTag
Design::getTag(Rsyn::LibraryCell libraryCell) {
	return LibraryCellTag(&libraryCell->tag);
} // end method

////////////////////////////////////////////////////////////////////////////////
// Range-Based Loop
////////////////////////////////////////////////////////////////////////////////

inline
Range<ListCollection<LibraryCellData, LibraryCell>>
Design::allLibraryCells(const bool showDeprecatedMessage) {
	if (showDeprecatedMessage) {
		std::cout << "WARNING: Rsyn::Desing::allLibraryCells() is deprecated. "
				"Use Rsyn::Library::allLibraryCells() instead.\n";
	} // end method
	return ListCollection<LibraryCellData, LibraryCell>(data->libraryCells);
} // end method

} // end namespace
