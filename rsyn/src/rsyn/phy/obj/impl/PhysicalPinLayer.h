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
 * Created on 18 de Setembro de 2016, 08:46
 */
namespace Rsyn {

inline const std::vector<Bounds> & PhysicalPinLayer::allBounds() const {
	return data->clsBounds;
} // end method 

// -----------------------------------------------------------------------------

inline const std::vector<Polygon> & PhysicalPinLayer::allPolygons() const {
	return data->clsPolygons;
} // end method 

// -----------------------------------------------------------------------------

inline PhysicalLayer PhysicalPinLayer::getLayer() const {
	return data->clsLibLayer;
} // end method 

// -----------------------------------------------------------------------------

inline std::size_t PhysicalPinLayer::getNumBounds() const {
	return data->clsBounds.size();
} // end method 

// -----------------------------------------------------------------------------

inline std::size_t PhysicalPinLayer::getNumPolygons() const {
	return data->clsPolygons.size();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPinLayer::hasRectangleBounds() const {
	return !data->clsBounds.empty();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPinLayer::hasPolygonBounds() const {
	return !data->clsPolygons.empty();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPinLayer::isRectangleBoundsEmpty() const {
	return data->clsBounds.empty();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPinLayer::isPolygonBoundsEmpty() const {
	return data->clsPolygons.empty();
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalPinLayer::hasLayer() const {
	return data->clsLibLayer != nullptr;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 