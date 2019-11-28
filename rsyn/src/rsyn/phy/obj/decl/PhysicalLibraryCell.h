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
 * File:   PhysicalLibraryCell.h
 * Author: jucemar
 *
 * Created on 12 de Setembro de 2016, 22:10
 */

#ifndef PHYSICALDESIGN_PHYSICALLIBRARYCELL_H
#define PHYSICALDESIGN_PHYSICALLIBRARYCELL_H

namespace Rsyn {

class PhysicalLibraryCell : public Proxy<PhysicalLibraryCellData> {
	friend class PhysicalDesign;
protected:
	//! @brief Constructs a Rsyn::PhysicalLibraryCell object with a pointer to Rsyn::PhysicalLibraryCellData.
	PhysicalLibraryCell(PhysicalLibraryCellData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalLibraryCell object with a null pointer to Rsyn::PhysicalLibraryCellData.
	PhysicalLibraryCell() : Proxy(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalLibraryCell object with a null pointer to Rsyn::PhysicalLibraryCellData.
	PhysicalLibraryCell(std::nullptr_t) : Proxy(nullptr) {}
	
	//! @brief Returns the PhysicalLibraryCell macro. 
	lefiMacro* getMacro() const;
	//! @brief Returns the PhysicalLibraryCell size. 
	//! @brief In dimension X is the PhysicalLibraryCell width while in Y is the PhysicalLibraryCell height.
	DBUxy getSize() const;
	//! @brief Returns a DBU type that represents the PhysicalLibraryCell width.
	DBU getWidth() const;
	//! @brief Returns a DBU type that represents the PhysicalLibraryCell height.
	DBU getHeight() const;
	//! @brief Returns a DBU type that represents the PhysicalLibraryCell length in Dimension parameter dim.
	DBU getLength(const Dimension dim) const;
	//! @brief Returns the library cell bounds.
	Bounds getBounds() const;
	//! @brief Returns true if the PhysicalLibraryCell has defined rectangular obstacle Bounds.
	bool hasLayerObstacles() const;
	bool hasObstacles() const;
	//! @brief Returns PhysicalSite object related to the PhysicalLibraryCell.
	Rsyn::PhysicalSite getSite() const;
	//! @breif Returns true if there is a top layer obstacle. The top layer obs is the highest metal layer blockage.
	bool hasTopLayerObstacle() const;
	//! @breif Returns the top layer obstacle. The top layer obs is the highest metal layer blockage.
	Rsyn::PhysicalObstacle getTopLayerObstracle() const;
	//! @brief Returns a constant vector reference to PhysicalObstacle objects that 
	//! represent the  metal layers blocked in the top of the PhysicalLibraryCell.
	const std::vector<Rsyn::PhysicalObstacle> & allObstacles() const ;
	//! @brief Returns the total number of metal layer (PhysicalObstacle) blocked in the top of the PhysicalLibraryCell.
	std::size_t getNumObstacles() const; 
	
	//! @brief Returns the number of cell layer boundaries. 
	//! @details In 2015 ICCAD contest, the boundaries for some macros are defined by one of the PhysicalLayer, 
	//! i.e., the PhysicalLibaryCellboundaries are defined by metal1 boundaries. 
	//! return 0  if the cell boundaries is not defined by one of layer metal. 
	//! Otherwise returns the number of rectangles that composes the cell boundary
	//! @warning This method affects 2015 ICCAD contest benchmark.
	std::size_t getNumPhysicalCellLayerBoundaries() const; 
	//! @brief Returns the PhysicalObstacle  that defines the cell boundary for 2015 ICCAD contest benchmark.
	//! @details In 2015 ICCAD contest, the boundaries for some macros are defined by one of the PhysicalLayer, 
	//! i.e., the PhysicalLibaryCellboundaries are defined by metal1 boundaries. 
	//! @warning This method affects 2015 ICCAD contest benchmark.
	Rsyn::PhysicalObstacle getLayerObstacles() const;
	//! @brief Returns a transformation that allows one to transform the
	//! coordinates according to a cell orientation.
	PhysicalTransform getTransform(const Rsyn::PhysicalOrientation &orientation) const;
	//! @brief Returns a constant reference vector to all Bounds that defines the cell boundary for 2015 ICCAD contest benchmark.
	//! @details In 2015 ICCAD contest, the boundaries for some macros are defined by one of the PhysicalLayer, 
	//! i.e., the PhysicalLibaryCellboundaries are defined by metal1 boundaries. 
	//! @warning This method affects 2015 ICCAD contest benchmark.
	const std::vector<Bounds> & allLayerObstacles() const;
	
	
	
	bool hasPolygonBoundaries() const;
	const Polygon & getPolygonBoundaries() const;
	
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALLIBRARYCELL_H */

