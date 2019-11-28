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
 * File:   PhysicalGroup.h
 * Author: jucemar
 *
 * Created on 09 de Abril de 2017, 16:45
 */

#ifndef PHYSICALDESIGN_PHYSICALGROUP_H
#define PHYSICALDESIGN_PHYSICALGROUP_H

namespace Rsyn {

class PhysicalGroup : public Proxy<PhysicalGroupData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::PhysicalGroup object with a pointer to Rsyn::PhysicalGroupData.

	PhysicalGroup(PhysicalGroupData * data) : Proxy(data) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalGroup object with a null pointer to Rsyn::PhysicalGroupData.

	PhysicalGroup() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalGroup object with a null pointer to Rsyn::PhysicalGroupData.

	PhysicalGroup(std::nullptr_t) : Proxy(nullptr) {
	}

	//! @brief Returns region's name
	const std::string & getName() const;
	//! @brief Returns the PhysicalRegion object associated to PhysicalGroup.
	Rsyn::PhysicalRegion getPhysicalRegion() const;
	//! @brief Returns the constant reference to the Region patters vector.
	const std::vector<std::string> & allPatterns() const;

}; // end class 

} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALGROUP_H */

