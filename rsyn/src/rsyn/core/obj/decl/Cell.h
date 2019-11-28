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

//! @brief A proxy class representing a netlist cell.
class Cell : public Instance {
friend class RawPointer;
friend class Design;
friend class Instance;
private:	
	Cell(InstanceData * data) : Instance(data) {}
public:

	//! @brief Default constructor.
	Cell() : Instance(nullptr) {}

	//! @brief Assignment constructor to allow null values.
	Cell(std::nullptr_t) : Instance(nullptr) {}

	//! @brief Returns the name of the library cell associated to this cell.
	const std::string &getLibraryCellName() const;

	//! @brief Returns the library cell associated to this cell.
	LibraryCell getLibraryCell() const;

	//! @brief Returns the respective pin in this cell associated to a library
	//!        pin if any.
	Pin getPinByLibraryPin(LibraryPin lpin) const;

	//! @brief Changes the library cell of this cell. The new library cell must
	//!        have the same interface (i.e. pin names and directions) as the
	//!        old one.
	void remap(LibraryCell libraryCell);

	//! @brief Changes the library cell of this cell. The new library cell must
	//!        have the same interface (i.e. pin names and directions) as the
	//!        old one.
	void remap(const std::string &libraryCellName);
	
}; // end class
	
} // end namespace