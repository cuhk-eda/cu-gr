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
 * File:   PhysicalPin.h
 * Author: jucemar
 *
 * Created on 14 de Setembro de 2016, 21:26
 */

#ifndef PHYSICALDESIGN_PHYSICALPIN_H
#define PHYSICALDESIGN_PHYSICALPIN_H

namespace Rsyn {

class PhysicalPin : public Proxy<PhysicalPinData> {
	friend class PhysicalDesign;
protected:
	//! @brief Constructs a Rsyn::PhysicalPin object with a pointer to Rsyn::PhysicalPinData.
	PhysicalPin(PhysicalPinData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalPin object with a null pointer to Rsyn::PhysicalPinData.
	PhysicalPin() : Proxy(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalPin object with a null pointer to Rsyn::PhysicalPinData.
	PhysicalPin(std::nullptr_t) : Proxy(nullptr) {}

	//! @brief Returns the pin displacement. 
	//! @details The displacement are independent to abscissa (X) and ordinate (X).
	//! The origin to compute pin displacement is the Left-Lower cell point and it is assumed to be (0,0). 
	//! @warning It was assumed the pin has a fixed position in the cell.
	DBUxy getDisplacement () const;
	//! @brief Returns the pin displacement to the Dimension parameter dim.
	//! @details The displacement are independent to abscissa (X) and ordinate (X).
	//! The origin to compute pin displacement is the Left-Lower cell point and it is assumed to be (0,0). 
	//! @warning It was assumed the pin has a fixed position in the cell.
	DBU getDisplacement(const Dimension dim);
	//! @brief Returns a vector reference to PhysicalPinGeometry objects related to PhysicalPin. 
	std::vector<PhysicalPinGeometry> & allPinGeometries();
	//! @brief Returns a constant vector reference to PhysicalPinGeometry objects related to PhysicalPin.
	const std::vector<PhysicalPinGeometry> & allPinGeometries() const;
	//! @brief Returns the total number of PhysicalPinGeometry objects related to PhysicalPin.
	std::size_t getNumPinGeometries() const;
	//! @brief Returns true if PhysicalPin has PhysicalPinGeometry objects. Otherwise, returns false.
	bool hasPinGeometries() const;
	//! @brief Returns true if PhysicalPin has no PhysicalPinGeometry objects. Otherwise, returns false.
	bool isEmptyPinGeometries() const;
	//! @brief Returns an enum of PhysicalPinDirection that gives the PhysicalPin direction.
	PhysicalPinDirection getDirection () const;
	//! @brief Returns a constant reference to the Bounds object that defines the PhysicalPin boundaries.
	//! it is is defined by one of the layer.
	const Bounds & getLayerBounds() const;
	
}; // end class 

} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALPIN_H */

