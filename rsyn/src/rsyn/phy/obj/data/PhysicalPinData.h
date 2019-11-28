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
 * File:   PhysicalPinData.h
 * Author: jucemar
 *
 * Created on 14 de Setembro de 2016, 21:24
 */

#ifndef PHYSICALDESIGN_PHYSICALPINDATA_H
#define PHYSICALDESIGN_PHYSICALPINDATA_H

namespace Rsyn {

class PhysicalPinData {
public:
	DBUxy clsDisplacement;
	//! @details A pin may have several ports. However, each port is weakly connected to other. 
	//! It is assumed that they have high resistance among them. 
	std::vector<PhysicalPinGeometry> clsPhysicalPinGeometries;
	PhysicalPinDirection clsDirection = PIN_INVALID_DIRECTION;
	Bounds clsLayerBound; // The pin bounds is defined by one of the layers
	PhysicalPinData() = default;
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALPINDATA_H */

