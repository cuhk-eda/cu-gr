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
 * File:   PhysicalLibraryPinData.h
 * Author: jucemar
 *
 * Created on 13 de Setembro de 2016, 21:04
 */

#ifndef PHYSICALDESIGN_PHYSICALLIBRARYPINDATA_H
#define PHYSICALDESIGN_PHYSICALLIBRARYPINDATA_H

namespace Rsyn {

class PhysicalLibraryPinData {
public:
	Bounds clsLayerBound; // The layer bound is defined by one of the layer.
	PhysicalLibraryCell clsLibraryCell;
	PhysicalPinUse clsUse = PIN_INVALID_USE;
	PhysicalPinDirection clsDirection = PIN_INVALID_DIRECTION;
	// A pin may have several ports. However, each port is weakly connected to other. 
	// It is assumed that they have high resistance among them. 
	std::vector<PhysicalPinGeometry> clsPhysicalPinGeometries;
	
	PhysicalLibraryPinData() = default;
}; // class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALLIBRARYPINDATA_H */

