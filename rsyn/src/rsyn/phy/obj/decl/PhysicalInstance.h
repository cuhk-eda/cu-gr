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
 * File:   PhysicalInstance.h
 * Author: jucemar
 *
 * Created on 4 de Outubro de 2016, 13:22
 */

#ifndef PHYSICALDESIGN_PHYSICALINSTANCE_H
#define PHYSICALDESIGN_PHYSICALINSTANCE_H

namespace Rsyn {
class PhysicalInstance : public Proxy<PhysicalInstanceData> {
	friend class PhysicalDesign;
protected:
	//! @brief Constructs a Rsyn::PhysicalInstance object with a pointer to Rsyn::PhysicalInstanceData.
	PhysicalInstance(PhysicalInstanceData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalInstance object with a null pointer to Rsyn::PhysicalInstanceData.
	PhysicalInstance()  {}
	//! @brief Constructs a Rsyn::PhysicalInstance object with a null pointer to Rsyn::PhysicalInstanceData.
	PhysicalInstance(std::nullptr_t)  {}
	
	//! @brief Returns the Instance associated to PhysicalInstance 
	Rsyn::Instance getInstance() const;
	//! @brief Returns the name of this instance.
	const std::string &getName() const;
	//! @brief Returns the instance area in DBU.
	DBU getArea() const;
	//! @brief Returns the central point of the PhysicalInstance boundaries.
	DBUxy getCenter() const;
	//! @brief Returns the central position of the PhysicalInstance boundaries for the given dimension.
	DBU getCenter(const Dimension dim) const;
	//! @brief Returns the x position of the instance.
	DBU getX() const;
	//! @brief Returns the y position of the instance.
	DBU getY() const;
	//! @brief Returns the Instance height. It is the length of PhysicalInstance boundaries is Y dimension.
	DBU getHeight() const;
	//! @brief Returns the Instance width. It is the length of PhysicalInstance boundaries is X dimension.
	DBU getWidth() const;
	//! @brief Returns the length of PhysicalInstance boundaries for given dimension
	DBU getSize(const Dimension dimension) const;
	//! @brief Returns the length of PhysicalInstance boundaries. 
	//! In X is the length for abscissa (width) while in Y is the length for ordinate (height).
	DBUxy getSize() const;
	//! @brief Returns the left-lower PhysicalInstance point that is defined as its position.
	DBUxy getPosition() const;
	//! @brief Returns the left-lower PhysicalInstance point for the given dimension 
	//! that is defined as its position.
	DBU getPosition(const Dimension dim) const;
	//! @brief Returns the PhysicalInstance point for the given boundary (LOWER or UPPER).
	DBUxy getCoordinate(const Boundary bound) const;
	//! @brief Returns the PhysicalInstance point for the given boundary (LOWER or UPPER) and dimension (X or Y).
	DBU getCoordinate(const Boundary bound, const Dimension dim) const;
	//! @brief Returns the orientation of the cell.
	PhysicalOrientation getOrientation() const;
	//! @brief Returns the bound box Bounds that defines the limits of PhysicalInstance.
	const Bounds &getBounds() const;
	
	Rsyn::PhysicalCell asPhysicalCell() const;
	
	// Mateus and Isadora @ 2018/03/19
	// Maybe it should be moved to PhysicalInstanceData?
	// Used in map-like data structures.
	friend bool operator<(const PhysicalInstance &left, const PhysicalInstance &right) {
		// [IMPORTANT] We don't use the pointer (e) directly to avoid
		// non-determinism behavior. Note that the pointer address may change
		// from execution to execution and hence the mapping function may return
		// elements in different order leading to different results.
		return left.getInstance() < right.getInstance();
	} // end method
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALINSTANCE_H */

