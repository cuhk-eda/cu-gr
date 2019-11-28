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
 
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PhysicalRegion.h
 * Author: jucemar
 *
 * Created on 09 de Abril de 2017, 14:55
 */

#ifndef PHYSICALDESIGN_PHYSICALREGION_H
#define PHYSICALDESIGN_PHYSICALREGION_H

namespace Rsyn {

class PhysicalRegion : public Proxy<PhysicalRegionData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::PhysicalRegion object with a pointer to Rsyn::PhysicalRegionData.

	PhysicalRegion(PhysicalRegionData * data) : Proxy(data) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalRegion object with a null pointer to Rsyn::PhysicalRegionData.

	PhysicalRegion() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalRegion object with a null pointer to Rsyn::PhysicalRegionData.

	PhysicalRegion(std::nullptr_t) : Proxy(nullptr) {
	}

	//! @brief Returns region's name
	const std::string & getName() const;
	//! @brief Returns region's type that may be FENCE or GUIDE.
	RegionType getType() const;
	//! @brief Returns the constant reference to the boundaries vector.
	const std::vector<Bounds> & allBounds() const;
	//! @brief Returns total number of boundaries. 
	std::size_t getNumBounds() const;

}; // end class 

} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALREGION_H */

