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
 
#ifndef PHYSICALGCELLDATA_H
#define PHYSICALGCELLDATA_H

namespace Rsyn {

class PhysicalGCellData {
	friend class PhysicalDesign;
public:
	//! @brief Direction of the GCell
	//! HORIZONTAL is equal to Y in DEF and VERTICAL is equal to X in DEF
	PhysicalGCellDirection clsDirection;
	//! @brief Each GCELLGRID statement specifies the location of the first vertical (x) and first horizontal (y) track. Source: DEF Manual.
	DBU clsLocation = 0;
	//! @brief Specifies the number of columns in the grid if Direction is VERTICAL
	//! Specifies the number of rows in the grid if Direction is HORIZONTAL
	int clsNumTracks = 0;
	//! @brief Specifies the spacing between tracks.
	DBU clsStep = 0;
	//! @brief	default constructor
	PhysicalGCellData() = default;
}; // end class 
} // end namespace 

#endif /* PHYSICALGCELLDATA_H */

