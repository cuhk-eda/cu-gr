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

//! @brief A proxy class representing a library module
//! @note  This is not used yet.
class LibraryModule : public Proxy<LibraryModuleData> {
	
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;

private:
	LibraryModule(LibraryModuleData * data) : Proxy(data) {}

public:

	//! @brief Default constructor.
	LibraryModule() {}

	//! @brief Assignment constructor to allow null values.
	LibraryModule(std::nullptr_t) {}	

	//! @brief Returns the design in which this library module was created.
	Design getDesign();

	//! @brief Returns the design in which this library module was created.
	const Design getDesign() const;	
	
}; // end class
	
} // end namespace