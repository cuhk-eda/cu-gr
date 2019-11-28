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
 * File:   PhysicalPinGeometryData.h
 * Author: jucemar
 *
 * Created on 13 de Setembro de 2016, 21:10
 */

#ifndef PHYSICALDESIGN_PHYSICALPINGEOMETRYDATA_H
#define PHYSICALDESIGN_PHYSICALPINGEOMETRYDATA_H

namespace Rsyn {

class PhysicalPinGeometryData : public PhysicalObject {
public:
	//! @brief An enum PhysicalPinGeometryClass that determines the class of the PhysicalPinGeometry object.
	PhysicalPinGeometryClass clsPinPortClass = PINGEOMETRYCLASS_NONE;
	//! @brief PhysicalPinLayers associated to the PhysicalPinGeometry.
	std::vector<PhysicalPinLayer> clsPinLayers;
	//! @brief Default constructor.
	//! @details Users do not have access to this constructor. The reference to 
	//! PhysicalPinGeometryData is a protected variable in PhysicalPinGeometry.

	PhysicalPinGeometryData() = default;

}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALPINGEOMETRYDATA_H */

