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
 * File:   PhysicalPinLayer.h
 * Author: jucemar
 *
 * Created on 13 de Setembro de 2016, 21:19
 */

#ifndef PHYSICALDESIGN_PHYSICALPINLAYER_H
#define PHYSICALDESIGN_PHYSICALPINLAYER_H

namespace Rsyn {

class PhysicalPinLayer : public Proxy<PhysicalPinLayerData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
protected:
	//! @brief Constructs a Rsyn::PhysicalPinLayer object with a pointer to Rsyn::PhysicalPinLayerData.
	PhysicalPinLayer(PhysicalPinLayerData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalPinLayer object with a null pointer to Rsyn::PhysicalPinLayerData.
	PhysicalPinLayer() : Proxy(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalPinLayer object with a null pointer to Rsyn::PhysicalPinLayerData.
	PhysicalPinLayer(std::nullptr_t) : Proxy(nullptr) {}

	//! @brief Returns the vector reference of rectangular bounds to the Rsyn::PhysicalPinLayer.
	const std::vector<Bounds> & allBounds() const;
	//! @brief Returns the vector reference of polygons to the PhysicalPinLayer.
	const std::vector<Polygon> & allPolygons() const;
	//! @brief Returns the Rsyn::PhysicalLayer object related to the  Rsyn::PhysicalPinLayer.
	Rsyn::PhysicalLayer getLayer() const;
	//! @brief Returns the total number of rectangular Bounds that defines Rsyn::PhysicalPinLayer
	std::size_t getNumBounds() const;
	//! @brief Returns the total number of Polygons that defines Rsyn::PhysicalPinLayer
	std::size_t getNumPolygons() const;
	//! @brief Returns true if the Rsyn::PhysicalPinLayer has rectangular Bounds.
	//! Otherwise, returns false.
	bool hasRectangleBounds() const;
	//! @brief Returns true if the Rsyn::PhysicalPinLayer has Polygons.
	//! Otherwise, returns false.
	bool hasPolygonBounds() const;
	//! @brief Returns true if the Rsyn::PhysicalPinLayer has no rectangular Bounds.
	//! Otherwise, returns false.
	bool isRectangleBoundsEmpty() const;
	//! @brief Returns true if the Rsyn::PhysicalPinLayer has no Polygons.
	//! Otherwise, returns false.
	bool isPolygonBoundsEmpty() const;
	//! @brief Returns true if a Rsyn::PhysicalLayer object is associated to the Rsyn::PhysicalPinLayer.
	//! Otherwise, returns false.
	bool hasLayer() const;
	
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALPINLAYER_H */

