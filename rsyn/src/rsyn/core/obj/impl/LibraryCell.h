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
LibraryCell::getDesign() {
	return data->design;
} // end method

// -----------------------------------------------------------------------------

inline
const Design 
LibraryCell::getDesign() const {
	return data->design;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryPin 
LibraryCell::getLibraryPinByName(const std::string &name) const {
	for (LibraryPin lpin : data->pins) {
		if (lpin.getName() == name) {
			return lpin;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
LibraryCell::getName() const {
	return data? data->name : NullName;
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryCell::getNumPins() const {
	return (int) data->pins.size();
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryCell::getNumPins(const Direction direction) const {
	// TODO: use array
	switch(direction) {
		case IN : return data->numInputPins;
		case OUT: return data->numOutputPins;
		case BIDIRECTIONAL: return data->numInOutPins;
		default: throw std::string("invalid pin direction"); // TODO: use a proper exception class
	} // end switch
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryCell::getNumInputPins() const {
	return getNumPins(Rsyn::IN);
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryCell::getNumOutputPins() const {
	return getNumPins(Rsyn::OUT);
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryCell::getNumBidirectionalPins() const  {
	return getNumPins(Rsyn::BIDIRECTIONAL);
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryCell::getNumArcs() const {
	return (int) data->arcs.size();
} // end method

// -----------------------------------------------------------------------------
	
inline
LibraryPin 
LibraryCell::getLibraryPinByIndex(const int index) const {
	return data->pins[index];
} // end method

// -----------------------------------------------------------------------------

inline
LibraryPin
LibraryCell::getAnyInputLibraryPin() {
	for (LibraryPin lpin : allLibraryPins()) {
		if (lpin.getDirection() == Rsyn::IN) {
			return lpin;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryPin
LibraryCell::getAnyOutputLibraryPin() {
	for (LibraryPin lpin : allLibraryPins()) {
		if (lpin.getDirection() == Rsyn::OUT) {
			return lpin;
		} // end if
	} // end for
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryArc 
LibraryCell::getLibraryArc(const Rsyn::LibraryPin from, const Rsyn::LibraryPin to) const {
	for (LibraryArc larc : allLibraryArcs()) {
		if (larc.getFromLibraryPin() == from && larc.getToLibraryPin() == to) 
			return larc;
	} // end method
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryArc
LibraryCell::getLibraryArcByIndex(const int index) const {
	return data->arcs[index];
} // end method

// -----------------------------------------------------------------------------

inline
LibraryArc 
LibraryCell::getLibraryArcByPinNames(const std::string &from, const std::string &to) const {
	for (LibraryArc larc : allLibraryArcs()) {
		if (larc.getFromName() == from && larc.getToName() == to) 
			return larc;
	} // end method
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryArc 
LibraryCell::getAnyLibraryArc() const {
	return getNumArcs() > 0? data->arcs.front() : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfLibraryPinsFilteredByDirection>
LibraryCell::allLibraryPins(const Direction direction) const {
	return CollectionOfLibraryPinsFilteredByDirection(data->pins, direction);
} // end method

// -----------------------------------------------------------------------------

inline
Range<CollectionOfLibraryPins>
LibraryCell::allLibraryPins() const {
	return CollectionOfLibraryPins(data->pins);
} // end method

// -----------------------------------------------------------------------------
		
inline
Range<CollectionOfLibraryArcs>
LibraryCell::allLibraryArcs() const {
	return CollectionOfLibraryArcs(data->arcs);
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryCell::isSequential() const {
	if (data->tag.logicType == LOGIC_TYPE_TAG_NOT_SPECIFIED)
		throw TagNotSpecifiedException("Logic Type");
	return data->tag.logicType == LOGIC_TYPE_TAG_SEQUENTIAL;
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryCell::isCombinational() const {
	if (data->tag.logicType == LOGIC_TYPE_TAG_NOT_SPECIFIED)
		throw TagNotSpecifiedException("Logic Type");
	return data->tag.logicType == LOGIC_TYPE_TAG_COMBINATIONAL;
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryCell::isTie(const TieType type) const {
	if (data->tag.tieType == TIE_TYPE_TAG_NOT_SPECIFIED)
		throw TagNotSpecifiedException("Tie Type");

	const TieTypeTag &tag = data->tag.tieType;
	switch (type) {
		case TIE_LOW     : return tag == TIE_TYPE_TAG_LOW;
		case TIE_HIGH    : return tag == TIE_TYPE_TAG_HIGH;
		case ANY_TIE_TYPE: return tag == TIE_TYPE_TAG_LOW || tag == TIE_TYPE_TAG_HIGH;
		default:
			assert(false);
			return false;
	} // end switch
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryCell::isBuffer(const BufferType type) const {
	if (data->tag.bufferType == BUFFER_TYPE_TAG_NOT_SPECIFIED)
		throw TagNotSpecifiedException("Buffer Type");

	const BufferTypeTag &tag = data->tag.bufferType;
	switch (type) {
		case NON_INVERTING  : return tag == BUFFER_TYPE_TAG_NON_INVERTING;
		case INVERTING      : return tag == BUFFER_TYPE_TAG_INVERTING;
		case ANY_BUFFER_TYPE: return tag == BUFFER_TYPE_TAG_NON_INVERTING || tag == BUFFER_TYPE_TAG_INVERTING;
		default:
			assert(false);
			return false;
	} // end switch
} // end method

// -----------------------------------------------------------------------------

inline DBU LibraryCell::getHeight() const {
	return data->size[Y];
} // end method

// -----------------------------------------------------------------------------

inline DBU LibraryCell::getWidth() const {
	return data->size[X];
} // end method

// -----------------------------------------------------------------------------

inline DBUxy LibraryCell::getSize() const {
	return data->size;
} // end method

// -----------------------------------------------------------------------------

inline DBU LibraryCell::getSize(const Dimension dimension) const {
	return data->size[dimension];
} // end method

// -----------------------------------------------------------------------------

inline DBU LibraryCell::getArea() const {
	return getWidth() * getHeight();
} // end method

// =============================================================================
// Tag
// =============================================================================

inline
LogicTypeTag
LibraryCellTag::getLogicType() const {
	return data->logicType;
} // end method

// -----------------------------------------------------------------------------

inline
TieTypeTag
LibraryCellTag::getTieType() const {
	return data->tieType;
} // end method

// -----------------------------------------------------------------------------

inline
BufferTypeTag
LibraryCellTag::getBufferTypeTag() const {
	return data->bufferType;
} // end method

// -----------------------------------------------------------------------------

inline
void
LibraryCellTag::setLogicType(const LogicTypeTag value) {
	data->logicType = value;
} // end method

// -----------------------------------------------------------------------------

inline
void
LibraryCellTag::setTieType(const TieTypeTag value) {
	data->tieType = value;
} // end method

// -----------------------------------------------------------------------------

inline
void
LibraryCellTag::setBufferType(const BufferTypeTag value) {
	data->bufferType = value;
} // end method

} // end namespace