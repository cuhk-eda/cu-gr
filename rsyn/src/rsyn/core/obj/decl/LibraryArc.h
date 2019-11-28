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

//! @brief A proxy class representing a library arc.
class LibraryArc : public Proxy<LibraryArcData> {

RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;

friend class Design;
friend class Arc;
friend class Sandbox;
friend class SandboxArc;

private:
	LibraryArc(LibraryArcData * data) : Proxy(data) {}

public:

	//! @brief Default constructor.
	LibraryArc() {}

	//! @brief Assignment constructor to allow null values.
	LibraryArc(std::nullptr_t) {}	

	//! @brief Returns the design in which this library arc was created.
	Design getDesign();

	//! @brief Returns the design in which this library arc was created.
	const Design getDesign() const;	

	//! @brief Returns the name of this libary arc (e.g a -> o).
	std::string getName() const;

	//! @brief Returns the full name of this libary arc (e.g nand:a -> nand:o).
	std::string getFullName() const;

	//! @brief Returns the name of the "from" library pin.
	std::string getFromName() const;

	//! @brief Returns the name of the "to" library pin.
	std::string getToName() const;

	//! @brief Returns the library cell to which this library arc belongs to.
	LibraryCell getLibraryCell() const;

	//! @brief Returns the "from" library pin.
	LibraryPin getFromLibraryPin() const;

	//! @brief Returns the "to" library pin.
	LibraryPin getToLibraryPin() const;

	//! @brief Returns the index of this arc relative to the library cell.
	int getIndex() const;
}; // end class
	
} // end namespace