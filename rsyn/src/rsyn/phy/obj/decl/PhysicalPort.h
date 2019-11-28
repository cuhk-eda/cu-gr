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
 * File:   PhysicalPort.h
 * Author: jucemar
 *
 * Created on 4 de Outubro de 2016, 13:23
 */

#ifndef PHYSICALDESIGN_PHYSICALPORT_H
#define PHYSICALDESIGN_PHYSICALPORT_H


namespace Rsyn {

class PhysicalPort : public PhysicalInstance {
	friend class PhysicalDesign;
protected:
	//! @brief Constructs a Rsyn::PhysicalPort object with a pointer to Rsyn::PhysicalInstanceData.
	PhysicalPort(PhysicalInstanceData * data) : PhysicalInstance(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalPort object with a null pointer to Rsyn::PhysicalInstanceData.
	PhysicalPort() : PhysicalInstance(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalPort object with a null pointer to Rsyn::PhysicalInstanceData.
	PhysicalPort(std::nullptr_t) : PhysicalInstance(nullptr) {}

	// Methods for physical cell when a circuit pin (port) is mapped to it
	// In the Rsyn the circuit pins (ports) and cells are the same object. Therefore,
	// in the mapping to physical object they must be the same object.
	//! @brief Returns the object of the layer associated to the PhysicalPort
	PhysicalLayer getLayer() const;
	//! @brief Returns true if the PhysicalPort has an associated layer. Otherwise, returns false.
	bool hasLayer() const;
	
	PhysicalOrientation getOrientation() const;
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALPORT_H */

