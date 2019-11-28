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
 * File:   PhysicalDie.h
 * Author: jucemar
 *
 * Created on 4 de Fevereiro de 2017, 08:49
 */

namespace Rsyn {

inline DBUxy PhysicalDie::getCoordinate(const Boundary bound) const {
	return data->clsBounds[bound];
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalDie::getCoordinate(const Boundary bound, const Dimension dim) const {
	return data->clsBounds[bound][dim];
} // end method 

// -----------------------------------------------------------------------------

inline const Bounds & PhysicalDie::getBounds() const {
	return data->clsBounds;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalDie::getLength(const Dimension dim) const {
	return data->clsBounds.computeLength(dim);
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalDie::getPosition(const Boundary boundary) const {
	return data->clsBounds[boundary];
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalDie::getCenterPosition() const {
	return data->clsBounds.computeCenter();
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalDie::getArea() const {
	return data->clsBounds.computeArea();
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 


