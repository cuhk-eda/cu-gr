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
const std::string
Pin::getName() const {
	switch (getInstanceType()) {
		case Rsyn::CELL:
			return getLibraryPin().getName();
		case Rsyn::PORT:
			return getInstanceName();
		case Rsyn::MODULE:
			return getPort().getName();
	} // end switch
	return "<unknown>";
} // end method

// -----------------------------------------------------------------------------

inline
const std::string 
Pin::getFullName(const std::string::value_type separator) const {
	if (isPort()) {
		return data? getName() : NullName;
	} else {
		return data? (getInstance().getName() + separator + getName()) : NullName;
	} // end else
} // end if

// -----------------------------------------------------------------------------

inline
Design
Pin::getDesign() {
	// TODO: too many indirection (more inside cell), maybe we need to cache 
	// this.
	return getInstance().getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
const Design
Pin::getDesign() const {
	// TODO: too many indirection (more inside cell), maybe we need to cache 
	// this.
	return data->instance.getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
Instance
Pin::getInstance() const {
	return data->instance;
} // end method

// -----------------------------------------------------------------------------

inline
Net
Pin::getNet() const {
	return data->net;
} // end method

// -----------------------------------------------------------------------------

inline
Direction
Pin::getDirection() const {
	return data->direction; 
} // end method

// -----------------------------------------------------------------------------

inline
int 
Pin::getIndex() const {
	return data->index;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string
Pin::getInstanceName() const {
	return getInstance().getName();
} // end method

// -----------------------------------------------------------------------------

inline
InstanceType
Pin::getInstanceType() const {
	return data->type;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
Pin::getNetName() const {
	return getNet().getName();
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
Pin::getDirectionName() const {
	return Global::getDirectionName(getDirection());
} // end method

// -----------------------------------------------------------------------------

inline
LibraryPin 
Pin::getLibraryPin() const {
	Rsyn::LibraryCell lcell = getLibraryCell();
	return lcell? lcell.getLibraryPinByIndex(data->index) : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell 
Pin::getLibraryCell() const {
	return getInstanceType() == Rsyn::CELL? 
		data->instance->lcell : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Port
Pin::getPort() const {
	switch (getInstanceType()) {
		case Rsyn::MODULE: 
			return getInstance().asModule().getPortByIndex(data->index);
		case Rsyn::PORT:
			return getInstance().asPort();
		default:
			return nullptr;
	} // end if
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isPort() const {
	return data->boundary;
} // end method


// -----------------------------------------------------------------------------

inline
bool
Pin::isPort(const Direction direction) const {
	return isPort() && (getPort().getDirection() == direction);
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Pin::isPortToUpLevelHierarchy() const {
	return data->boundary && data->type == Rsyn::PORT;
} // end method
// -----------------------------------------------------------------------------

inline
bool 
Pin::isPortToDownLevelHierarchy() const {
	return data->boundary && data->type == Rsyn::MODULE;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isConnected() const {
	return getNet();
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Pin::isDisconnected() const {
	return !getNet();
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Pin::isInput() const {
	return getDirection() == Rsyn::IN;
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Pin::isOutput() const {
	return getDirection() == Rsyn::OUT;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isBidirectional() const {
	return getDirection() == Rsyn::BIDIRECTIONAL;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isDriver() const {
	return isOutput();
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isSink() const {
	return isInput();
} // end method

// -----------------------------------------------------------------------------

inline
Arc 
Pin::getArcTo(Pin to) {
	for (Arc arc : allOutgoingArcs()) {
		if (arc.getToPin() == to) {
			return arc;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Arc 
Pin::getArcFrom(Pin from) {
	for (Arc arc : allIncomingArcs()) {
		if (arc.getFromPin() == from) {
			return arc;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------
inline
int 
Pin::getNumIncomingArcs() const {
	return (int) data->arcs[BACKWARD].size();
} // end method

// -----------------------------------------------------------------------------

inline
int 
Pin::getNumOutgomingArcs() const {
	return (int) data->arcs[FORWARD].size();
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPredecessorPins>
Pin::allPredecessorPins(const bool crossBoundaries) const {
	return CollectionOfPredecessorPins(*this, crossBoundaries);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfSuccessorPins>
Pin::allSucessorPins(const bool crossBoundaries) const {
	return CollectionOfSuccessorPins(*this, crossBoundaries);
} // end method

// -----------------------------------------------------------------------------

inline
const std::vector<Arc> &
Pin::allIncomingArcs() const {
	return data->arcs[BACKWARD];
} // end method

// -----------------------------------------------------------------------------

inline
const std::vector<Arc> &
Pin::allOutgoingArcs() const {
	return data->arcs[FORWARD];
} // end method

// -----------------------------------------------------------------------------

inline
const std::vector<Arc> &
Pin::allArcs(const TraverseType direction) const {
	return data->arcs[direction];
} // end method

// -----------------------------------------------------------------------------

inline
void 
Pin::connect(Net net) {
	getDesign().connectPin(*this, net);
} // end method

// -----------------------------------------------------------------------------

inline
void 
Pin::disconnect() {
	getDesign().disconnectPin(*this);
} // end method
	
// -----------------------------------------------------------------------------

inline
TopologicalIndex
Pin::getTopologicalIndex() const {
	return data->order;
} // end method


// -----------------------------------------------------------------------------

inline
bool
Pin::isMacroBlockPin() const {
	return data->instance.isMacroBlock();
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isConnectedToClockNetwork() const {
	return data->net? data->net.isClockNetwork() : false;
} // end method

// -----------------------------------------------------------------------------

inline
Use
Pin::getUse() const {
	Rsyn::LibraryPin lpin = getLibraryPin();
	
	if (!lpin) {
		return Rsyn::UNKNOWN_USE;
	} // end if
	
	return lpin.getUse();
} // end method

// -----------------------------------------------------------------------------

inline
bool
Pin::isPowerOrGround() const {
	Rsyn::LibraryPin lpin = getLibraryPin();
	
	if (!lpin) {
		return false;
	} // end if
	
	return (lpin.getUse() == POWER || lpin.getUse() == GROUND);
} // end method

} // end namespace