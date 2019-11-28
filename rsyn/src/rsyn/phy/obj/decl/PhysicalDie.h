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
 * Created on 4 de Fevereiro de 2017, 08:45
 */

#ifndef PHYSICALDESIGN_PHYSICALDIE_H
#define PHYSICALDESIGN_PHYSICALDIE_H

namespace Rsyn {

class PhysicalDie : public Proxy<PhysicalDieData> {
	friend class PhysicalDesign;
protected:
	PhysicalDie(PhysicalDieData * data) : Proxy(data) {}
public:
	//! @brief	Default constructor. Rectangular bounds are initialized to Bounds(DBUxy(0,0), DBUxy(0,0)).
	//!		PhysicalDieData pointer is initialized to nullptr.		
	//! @note	Die is assumed to be a rectangle. It does not support die defined as a polygon.
	PhysicalDie()  {}
	//! @brief 
	PhysicalDie(std::nullptr_t)  {}

	//! @brief	Returns the coordinate (x,y) of the boundary Rsyn::LOWER or Rsyn::UPPER of the die.
	DBUxy getCoordinate(const Boundary bound) const;
	//! @brief	Returns the coordinate X or Y of the boundary Rsyn::LOWER or Rsyn::UPPER of the die.
	DBU getCoordinate(const Boundary bound, const Dimension dim) const;
	//! @brief	Returns constant reference to Bounds objects.
	const Bounds & getBounds() const;
	//! @brief	Returns the dimension length for abscissa (X) or ordinate (Y).
	DBU getLength(const Dimension dim) const;
	//! @brief	Returns the coordinate X or Y of the boundary LOWER or UPPER of the die.
	DBUxy getPosition(const Boundary boundary = LOWER) const;
	//! @brief	Returns the central position (X, Y) in DBU units of the die bounds.
	DBUxy getCenterPosition() const;
	//! @brief	Returns total area of the PhysicalDie Bounds.
	DBU getArea() const;
}; //end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALDIE_H */

