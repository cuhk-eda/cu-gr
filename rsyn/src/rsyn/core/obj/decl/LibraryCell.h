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

//! @brief A proxy class representing a library cell.
class LibraryCell : public Proxy<LibraryCellData> {

RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;
friend class Pin;
friend class Cell;
friend class Sandbox;
friend class SandboxPin;
friend class SandboxCell;
friend class PhysicalDesign;

private:
	LibraryCell(LibraryCellData * data) : Proxy(data) {}

public:

	//! @brief Default constructor.
	LibraryCell() {}

	//! @brief Assignment constructor to allow null values.
	LibraryCell(std::nullptr_t) {}	

	//! @brief Returns the design in which this library cell was created.
	Design getDesign();

	//! @brief Returns the design in which this library cell was created.
	const Design getDesign() const;	

	//! @brief Returns the name of this library cell.
	const std::string &getName() const;

	//! @brief Returns the number of interface pin.
	int getNumPins() const;

	//! @brief Returns the number of interface pin of a given direction.
	int getNumPins(const Direction direction) const;

	//! @brief Returns the number of interface input pin.
	int getNumInputPins() const;

	//! @brief Returns the number of interface output pin.
	int getNumOutputPins() const;

	//! @brief Returns the number of interface bidirectional pin.
	int getNumBidirectionalPins() const ;	

	//! @brief Returns the number of arcs.
	int getNumArcs() const;

	//! @brief Returns a library pin by its index.
	LibraryPin getLibraryPinByIndex(const int index) const;

	//! @brief Returns a library pin by its name. If no pin matches, null is
	//!        returned.
	LibraryPin getLibraryPinByName(const std::string &name) const;

	//! @brief Returns any input library pin. Useful when handling
	//!        buffers/inverters, which have only one input pin.
	LibraryPin getAnyInputLibraryPin();

	//! @brief Returns any output library pin. Useful when handling 
	//!        buffers/inverters, which have only one output pin.
	LibraryPin getAnyOutputLibraryPin();

	//! @brief Returns the library arc from the "from" pin to the "to" pin. 
	//!        Returns null if the library pins do not belong to this instance
	//!        or if no such arc is found.
	LibraryArc getLibraryArc(const Rsyn::LibraryPin from, const Rsyn::LibraryPin to) const;

	//! @brief Returns the library arc by its index.
	LibraryArc getLibraryArcByIndex(const int index) const;

	//! @brief Returns the library arc from the "from" pin to the "to" pin. 
	//!        Returns null if either pin is not found or if no such arc exist 
	//!        between the pins.
	LibraryArc getLibraryArcByPinNames(const std::string &from, const std::string &to) const;

	//! @brief Returns any arc. Useful when handling buffers/inverters, which
	//!        have only one arc.
	LibraryArc getAnyLibraryArc() const;

	//! @brief Returns an iterable collection of all interface library pins of 
	//!        a given direction.
	Range<CollectionOfLibraryPinsFilteredByDirection>
	allLibraryPins(const Direction direction) const;

	//! @brief Returns an iterable collection of all interface library pins.
	Range<CollectionOfLibraryPins>
	allLibraryPins() const;		
	
	//! @brief Returns an iterable collection of all arcs.
	Range<CollectionOfLibraryArcs>
	allLibraryArcs() const;	

	//! @brief Returns true if this library cell represents a sequential cell 
	//!        and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isSequential() const;

	//! @brief Returns true if this library cell represents a combinational cell
	//!        and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isCombinational() const;

	//! @brief Returns true if this library cell represents a tie cell and false
	//!        otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isTie(const TieType type = ANY_TIE_TYPE) const;

	//! @brief Returns true if this library cell represents a buffer cell of a
	//!        given type and false otherwise.
	//! @note  This is a user-defined flag. If the flag was not set, an exception
	//!        is raised.
	bool isBuffer(const BufferType type) const;

	// Physical
	// --------

	//! @brief Returns the Instance height.
	DBU getHeight() const;

	//! @brief Returns the Instance width.
	DBU getWidth() const;

	//! @brief Returns the size of this library cell.
	DBUxy getSize() const;

	//! @brief Returns the size of the library cell in a given dimension.
	DBU getSize(const Dimension dimension) const;

	//! @brief Returns the area of this library cell.
	DBU getArea() const;

}; // end class

// =============================================================================
// Tag
// =============================================================================

//! @brief TODO
class LibraryCellTag : public Proxy<LibraryCellTagData> {
friend class Design;
private:
	LibraryCellTag(LibraryCellTagData * data) : Proxy(data) {}

public:

	LibraryCellTag() {}
	LibraryCellTag(std::nullptr_t) {}

	LogicTypeTag getLogicType() const;
	TieTypeTag getTieType() const;
	BufferTypeTag getBufferTypeTag() const;

	void setLogicType(const LogicTypeTag value);
	void setTieType(const TieTypeTag value);
	void setBufferType(const BufferTypeTag value);
}; // end class

} // end namespace