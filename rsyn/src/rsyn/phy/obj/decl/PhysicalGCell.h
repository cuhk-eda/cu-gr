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
 
#ifndef PHYSICALGCELL_H
#define PHYSICALGCELL_H

namespace Rsyn {

class PhysicalGCell : public Proxy<PhysicalGCellData> {
	friend class PhysicalDesign;
protected:
	PhysicalGCell(PhysicalGCellData * data) : Proxy(data) {}
public:
	//! @brief	Default constructor. 
	//!		PhysicalGCellData pointer is initialized to nullptr.		
	PhysicalGCell()  {}
	//! @brief 
	PhysicalGCell(std::nullptr_t)  {}
	
	//! @brief Returns GCell direction
	Rsyn::PhysicalGCellDirection getDirection() const;
	
	//! @brief If direction is vertical, then numTracks means the number of columns
	//! If direction is Horizontal, then numTracks means the number of rows
	int getNumTracks() const;
	//! @brief If direction is Horizontal, then location is Y value.
	//! If direction is vertical, then location is X value.
	DBU getLocation() const;

	//! @brief Returns the spacing between GCell tracks. 
	DBU getStep() const;
}; // end class 
} // end namespace 

#endif /* PHYSICALGCELL_H */

