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
 * File:   PhysicalCell.h
 * Author: jucemar
 *
 * Created on 12 de Setembro de 2016, 21:04
 */

#ifndef PHYSICALDESIGN_PHYSICALCELL_H
#define PHYSICALDESIGN_PHYSICALCELL_H

namespace Rsyn {

class PhysicalCell : public PhysicalInstance {
	friend class PhysicalDesign;
	friend class PhysicalInstance;
protected:
	PhysicalCell(PhysicalInstanceData * data) : PhysicalInstance(data) {}
public:
	PhysicalCell() : PhysicalInstance(nullptr) {}
	PhysicalCell(std::nullptr_t): PhysicalInstance(nullptr)   {} 

	//! @brief Returns the cell associated to this physical cell.
	Rsyn::Cell getCell() const;
	//! @brief Returns true if cell is fixed. Otherwise, returns false.
	bool isFixed() const;
	//! @brief Returns true if cell is MACRO type. Otherwise, returns false.
	bool isMacroBlock() const;
	//! @brief Returns true if cell is placed. Otherwise, returns false. 
	bool isPlaced() const;
	//! @brief Returns initial position of the cell. 
	//! @details Initial position is defined in the circuit placement files. 
	//! Otherwise, it is defined to the coordinate DBUxy(0,0);
	DBUxy getInitialPosition() const;
	//! @brief Returns initial position of the cell for the given dimension. 
	//! @details Initial position is defined in the circuit placement files. 
	//! Otherwise, it is defined to the coordinate DBUxy(0,0);
	DBU getInitialPosition(const Dimension dim) const;
	//! @brief Returns cell displacement. 
	//! @details Cell displacement is the difference between the current position of the cell 
	//! and the initial position of the cell.
	DBU getDisplacement() const;
	//! @brief Returns cell displacement. 
	//! @details Cell displacement is the difference between the current position of the cell 
	//! and the initial position of the cell.
	DBU getDisplacement(const Dimension dim) const;
	//! @brief Returns cell displacement for the given dimension. 
	//! @details Cell displacement is the difference between the current position of the cell 
	//! and the given position.
	DBU getDisplacement(const DBUxy pos) const;
	//! @brief Returns cell displacement to the given position and given dimension. 
	//! @details Cell displacement is the difference between the current position of the cell 
	//! and the given position in the given dimension.
	DBU getDisplacement(const DBU pos, const Dimension dim) const;
	//! @brief Returns cell displacement for the given dimension. 
	//! @details Cell displacement is the difference between the current position of the cell 
	//! and the given position.
	DBU getDisplacementFromCurrentPosition(const DBUxy pos) const;
	//! @brief Returns cell displacement to the given position and given dimension. 
	//! @details Cell displacement is the difference between the current position of the cell 
	//! and the given position in the given dimension.
	DBU getDisplacementFromCurrentPosition(const DBU pos, const Dimension dim) const;
	DBU getDisplacementFromInitialPosition(const DBUxy pos) const;
	DBU getDisplacementFromInitialPosition(const DBU pos, const Dimension dim) const;
	//! @brief Returns a transformation that allows one to transform the
	//! coordinates from the library cell space to the cell space. The
	//! transformation accounts for translation and the orientation of the cell.
	//! @param origin If set to true, the transform is created w.r.t (0, 0) and
	//! not the current cell position.
	PhysicalTransform getTransform(const bool origin = false) const;
	//! @brief Returns true if the cell boundaries is defined by a PhysicalLayer. Otherwise, returns false.
	//! @details Cell boundaries may be defined by some PhysicalLayer instead of rectangular Bounds.
	//! In the 2015 ICCAD contest, some macro have their boundaries defined by metal1 layer.
	bool hasLayerBounds() const; 
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALCELL_H */

