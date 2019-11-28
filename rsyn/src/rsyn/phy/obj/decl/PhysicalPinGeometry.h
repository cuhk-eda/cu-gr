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
 * File:   PhysicalPinGeometry.h
 * Author: jucemar
 *
 * Created on 13 de Setembro de 2016, 21:13
 */

#ifndef PHYSICALDESIGN_PHYSICALPINGEOMETRY_H
#define PHYSICALDESIGN_PHYSICALPINGEOMETRY_H

namespace Rsyn {

class PhysicalPinGeometry : public Proxy<PhysicalPinGeometryData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
protected:
	//! @brief Constructs a Rsyn::PhysicalPinGeometry object with a pointer to Rsyn::PhysicalPinGeometryData.
	PhysicalPinGeometry(PhysicalPinGeometryData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalPinGeometry object with a null pointer to Rsyn::PhysicalPinGeometryData.
	PhysicalPinGeometry() : Proxy(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalPinGeometry object with a null pointer to Rsyn::PhysicalPinGeometryData.
	PhysicalPinGeometry(std::nullptr_t) : Proxy(nullptr) {}
	
	//! @brief Returns the PhysicalPinGeometryClass type of the PhysicalPinLayer.
	//! @details A PhysicalPinGeometryClass may be: 1) PINGEOMETRYCLASS_NONE (default), 2) PINGEOMETRYCLASS_CORE, or PINGEOMETRYCLASS_BUMP.
	Rsyn::PhysicalPinGeometryClass getPinGeometryClass() const;
	//! @brief Returns true if a PhysicalPinLayer was associated to the PhysicalPinGeometry. 
	//! Otherwise, returns false.
	bool hasPinLayer() const ;
	
	//! @brief Returns the number if pin layers
	std::size_t getNumPinLayers() const;
	//! @brief Returns the PhysicalPinLayers associated to the PhysicalPinGeometry.
	const std::vector<Rsyn::PhysicalPinLayer> & allPinLayers() const;
	
	//! @brief Returns the lower pin layer. It is sorted by physical layer.
	//! If a pin layer is not defined, than returns a physical pin layer with nullptr data
	Rsyn::PhysicalPinLayer getLowerPinLayer() const;
	
	//! @brief Returns the upper pin layer. It is sorted by physical layer.
	//! If a pin layer is not defined, than returns a physical pin layer with nullptr data
	Rsyn::PhysicalPinLayer getUpperPinLayer() const;
}; // end class 

} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALPINGEOMETRY_H */

