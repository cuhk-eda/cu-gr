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

//! @brief A proxy class representing a netlist arc.
class Arc : public Proxy<ArcData> {
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
friend class RawPointer;
friend class Design;
private:
	Arc(ArcData * data) : Proxy(data) {}
	
public:

	//! @brief Default constructor.
	Arc() {}

	//! @brief Assignment constructor to allow null values.
	Arc(std::nullptr_t) {}

	//! @brief Returns the name of the arc (e.g. a -> o).
	std::string getName() const;

	//! @brief Returns the full name of the arc (e.g. u1:a -> u1:o).
	std::string getFullName() const;

	//! @brief Returns the name of the "from" pin.
	const std::string getFromName() const;

	//! @brief Returns the name of the "to" pin.
	const std::string getToName() const;	

	//! @brief Returns the design in which this arc is instantiated.
	Design getDesign();

	//! @brief Returns the design in which this arc is instantiated.
	const Design getDesign() const;

	//! @brief Returns the type of this arc (i.e. cell or net arc).
	ArcType getType() const;

	//! @brief Returns the instance to which this arc is associated if any. If
	//!        this is a net arc, returns null.
	Instance getInstance() const;

	//! @brief Returns the net to which this arc is associated if any. If
	//!        this is a cell arc, returns null.
	Net getNet() const;

	//! @brief Returns the library arc related to this arc if any. If this is a
	//!        net arc, returns null.
	LibraryArc getLibraryArc() const;

	//! @brief Returns the library cell related to this arc if any. If this is a
	//!        net arc or belong to an instance without library cell, returns
	//!        null.
	LibraryCell getLibraryCell() const;

	//! @brief Returns the "from" pin of this arc.
	//! @note  For cell arcs, the "from" pin is always an input pin. For net
	//!        arcs, the "from" pin is always an output (driver) pin.
	//!        Bidirectional pins are not handled yet.
	Pin getFromPin() const;
	
	//! @brief Returns the "to" pin of this arc.
	//! @note  For cell arcs, the "to" pin is always an output pin. For net
	//!        arcs, the "to" pin is always an input (sink) pin.
	//!        Bidirectional pins are not handled yet.
	Pin getToPin() const;

	//! @brief Returns the net to which the "from" pin connects to if any.
	Net getFromNet() const;

	//! @brief Returns the net to which the "to" pin connects to if any.
	Net getToNet() const;

	//! @brief Returns the index of this arc relative to its instance and
	//! library cell. If this arc does not belong to an instance (i.e. net arc),
	//! returns -1.
	int getIndex() const;
}; // end class
	
} // end namespace