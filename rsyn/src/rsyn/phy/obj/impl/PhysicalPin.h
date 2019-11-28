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
 * Created on 18 de Setembro de 2016, 09:00
 */

namespace Rsyn {

inline DBUxy PhysicalPin::getDisplacement() const {
	return data->clsDisplacement;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalPin::getDisplacement(const Dimension dim) {
	return data->clsDisplacement[dim];
} // end method 

// -----------------------------------------------------------------------------

inline std::vector<PhysicalPinGeometry> & PhysicalPin::allPinGeometries() {
	return data->clsPhysicalPinGeometries;
} // end method 

// -----------------------------------------------------------------------------

inline const std::vector<PhysicalPinGeometry> & PhysicalPin::allPinGeometries() const {
	return data->clsPhysicalPinGeometries;
} // end method 

// -----------------------------------------------------------------------------

inline std::size_t PhysicalPin::getNumPinGeometries() const {
	return data->clsPhysicalPinGeometries.size();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPin::hasPinGeometries() const {
	return !data->clsPhysicalPinGeometries.empty();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPin::isEmptyPinGeometries() const {
	return data->clsPhysicalPinGeometries.empty();
} // end method 

// -----------------------------------------------------------------------------

inline PhysicalPinDirection PhysicalPin::getDirection() const {
	return data->clsDirection;
} // end method 

// -----------------------------------------------------------------------------

inline const Bounds & PhysicalPin::getLayerBounds() const {
	return data->clsLayerBound;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 