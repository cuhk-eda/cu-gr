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
 * File:   PhysicalNet.h
 * Author: jucemar
 *
 * Created on 18 de Setembro de 2016, 09:03
 */

namespace Rsyn {

// -----------------------------------------------------------------------------

inline Rsyn::Net PhysicalNet::getNet() const {
	return data? data->clsNet : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline std::string PhysicalNet::getName() const {
	return getNet().getName();
} // end method

// -----------------------------------------------------------------------------

inline DBUxy PhysicalNet::getHPWL() const {
	return data->clsBounds.computeLength();
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalNet::getHPWL(const Dimension dim) {
	return data->clsBounds.computeLength(dim);
} // end method 

// -----------------------------------------------------------------------------

inline const Bounds & PhysicalNet::getBounds() const {
	return data->clsBounds;
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalNet::getCoordinate(const Boundary bound) const {
	return data->clsBounds[bound];
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalNet::getCoordinate(const Boundary bound, const Dimension dim) const {
	return data->clsBounds[bound][dim];
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::Pin PhysicalNet::getPinBoundary(const Boundary bound, const Dimension dim) const {
	return data->clsBoundPins[bound][dim];
} // end method 

// -----------------------------------------------------------------------------

inline const PhysicalRouting &PhysicalNet::getRouting() const {
	return data->clsRouting;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalNet::isRouted() const {
	return !data->clsRouting.isEmpty();
} // end method

// -----------------------------------------------------------------------------

} // end namespace 