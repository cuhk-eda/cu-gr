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
Net::_getDesign() const {
	// TODO: too slow...
	return _getParent().getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
Design
Net::getDesign() {
	return _getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
const Design
Net::getDesign() const {
	return _getDesign();
} // end method
	
// -----------------------------------------------------------------------------

inline
Module
Net::_getParent() const {
	return data->parent;
} // end method

// -----------------------------------------------------------------------------

inline
Module
Net::getParent() {
	return _getParent();
} // end method

// -----------------------------------------------------------------------------

inline
const Module
Net::getParent() const {
	return _getParent();
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
Net::getName() const {
	return data? getDesign()->netNames[data->id] : NullName;
} // end method

// -----------------------------------------------------------------------------

inline
int 
Net::getNumPins() const {
	return (int) data->pins.size();
} // end method

// -----------------------------------------------------------------------------

inline
int 
Net::getNumSinks() const {
	return data->numPinsOfType[SINK];
} // end method

// -----------------------------------------------------------------------------

inline
int 
Net::getNumDrivers() const {
	return data->numPinsOfType[DRIVER];
} // end method

// -----------------------------------------------------------------------------

inline
Pin 
Net::getAnyDriver() const {
	return data->driver;
} // end method

// -----------------------------------------------------------------------------

inline
Arc
Net::getArc(const Pin from, const Pin to) {
	for (Arc arc : allArcs()) {
		if ((arc.getFromPin() == from) && (arc.getToPin() == to)) {
			return arc;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
TopologicalIndex 
Net::getTopologicalIndex() const {
	TopologicalIndex order;

	if (hasMultipleDrivers()) {
		// Just to save runtime as this is not very common.
		for (Rsyn::Pin driver : allPins(Rsyn::DRIVER)) {
			order = driver.getTopologicalIndex();
		} // end for
	} else if (hasDriver()) {
		order = getAnyDriver().getTopologicalIndex();
	} else {
		// No driver...
		if (hasSink()) {
			order = +std::numeric_limits<TopologicalIndex>::infinity();
			for (Rsyn::Pin sink : allPins(Rsyn::SINK)) {
				order = std::min(order, sink.getTopologicalIndex());
			} // end for
		} else {
			// A floating ..
			order = MIN_TOPOLOGICAL_INDEX;
		} // end else
	} // end else
	
	return order;	
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Net::hasMultipleDrivers() const {
	return getNumDrivers() > 1;
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Net::hasSingleDriver() const {
	return getNumDrivers() == 1;
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Net::hasDriver() const {
	return getNumDrivers() >= 1;
} // end method

// -----------------------------------------------------------------------------

inline
bool 
Net::hasSink() const {
	return getNumSinks() >= 1;
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPins>
Net::allPins(bool filterPG) const {
	return CollectionOfPins(data->pins, filterPG);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfPinsFilteredByDirection>
Net::allPins(const Direction direction) const {
	return CollectionOfPinsFilteredByDirection(data->pins, direction);
} // end method

// -----------------------------------------------------------------------------
	
inline
Range<CollectionOfArcs>
Net::allArcs() const {
 	if (hasMultipleDrivers()) {
		throw Exception("Net::allArcs() does not support multiple drivers yet.");
	} else {
		if (hasDriver()) {
			return CollectionOfArcs(getAnyDriver()->arcs[FORWARD]);
		} else {
			static std::vector<Arc> emptyCollectionOfArcs; // dummy
			return CollectionOfArcs(emptyCollectionOfArcs);
		} // end else
	} // end else
} // end method

// -----------------------------------------------------------------------------
	
inline
bool
Net::isIdeal() const {
	if (data->tag.ideal.isNotSpecified())
		throw TagNotSpecifiedException("Ideal");
	return data->tag.ideal;
} // end method

// -----------------------------------------------------------------------------

inline
bool
Net::isClockNetwork() const {
	if (data->tag.type == NET_TYPE_TAG_NOT_SPECIFIED)
		throw TagNotSpecifiedException("Net Type");
	return data->tag.type == NET_TYPE_TAG_CLOCK;
} // end method

// -----------------------------------------------------------------------------

inline
NetTypeTag
Net::getNetTypeTag() const {
	return data->tag.type;
} // end method

// =============================================================================
// Tag
// =============================================================================

inline
TristateFlag
NetTag::getIdeal() const {
	return data->type;
} // end method

// -----------------------------------------------------------------------------

inline
NetTypeTag
NetTag::getType() const {
	return data->type;
} // end method

// -----------------------------------------------------------------------------

inline
void
NetTag::setType(const NetTypeTag value) {
	data->type = value;
} // end method

// -----------------------------------------------------------------------------

inline
void
NetTag::setIdeal(const bool value) {
	data->ideal = value;
} // end method

// -----------------------------------------------------------------------------

inline
Use
Net::getUse() const {
	return data->netUse;
} // end method

// -----------------------------------------------------------------------------

inline
void
Net::setUse(const Use use) {
	data->netUse = use;
} // end method

} // end namespace