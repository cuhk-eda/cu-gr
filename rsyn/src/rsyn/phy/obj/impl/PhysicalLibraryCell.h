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

namespace Rsyn {

inline lefiMacro* PhysicalLibraryCell::getMacro() const {
	return data->clsMacro;
} // end method 

//-----------------------------------------------------------------------------

inline DBUxy PhysicalLibraryCell::getSize() const {
	return data->clsSize;
} // end method 

//-----------------------------------------------------------------------------

inline DBU PhysicalLibraryCell::getWidth() const {
	return data->clsSize[X];
} // end  method 

//-----------------------------------------------------------------------------

inline DBU PhysicalLibraryCell::getHeight() const {
	return data->clsSize[Y];
} // end  method 

//-----------------------------------------------------------------------------

inline DBU PhysicalLibraryCell::getLength(const Dimension dim) const {
	return data->clsSize[dim];
} // end  method 

// -----------------------------------------------------------------------------

inline Bounds PhysicalLibraryCell::getBounds() const {
	return Bounds(0, 0, getWidth(), getHeight());
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalLibraryCell::hasLayerObstacles() const {
	return data->clsLayerBoundIndex > -1;
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalLibraryCell::hasObstacles() const {
	return !data->clsObs.empty();
} // end method 

// -----------------------------------------------------------------------------

inline const std::vector<Bounds> & PhysicalLibraryCell::allLayerObstacles() const {
	return data->clsObs[data->clsLayerBoundIndex].allBounds();
} // end method 

//-----------------------------------------------------------------------------

inline PhysicalSite PhysicalLibraryCell::getSite() const {
	return data->clsMacroSite;
} // end  method 

// -----------------------------------------------------------------------------

inline 	bool PhysicalLibraryCell::hasTopLayerObstacle() const{
	return data->clsTopLayerObs != nullptr;
} // end  method 

//-----------------------------------------------------------------------------
inline 	Rsyn::PhysicalObstacle PhysicalLibraryCell::getTopLayerObstracle() const {
	return data->clsTopLayerObs;
} // end  method 

//-----------------------------------------------------------------------------
		
inline const std::vector<PhysicalObstacle> & PhysicalLibraryCell::allObstacles() const {
	return data->clsObs;
} // end  method 

//-----------------------------------------------------------------------------

inline std::size_t PhysicalLibraryCell::getNumObstacles() const {
	return data->clsObs.size();
} // end  method 

//-----------------------------------------------------------------------------

inline std::size_t PhysicalLibraryCell::getNumPhysicalCellLayerBoundaries() const {
	if (data->clsLayerBoundIndex < 0)
		return 0;
	return data->clsObs[data->clsLayerBoundIndex].getNumObs();
} // end  method 

//-----------------------------------------------------------------------------

inline PhysicalObstacle PhysicalLibraryCell::getLayerObstacles() const {
	return data->clsObs[data->clsLayerBoundIndex];
} // end  method 

// -----------------------------------------------------------------------------

inline PhysicalTransform PhysicalLibraryCell::getTransform(const Rsyn::PhysicalOrientation &orientation) const {
	return PhysicalTransform(getBounds(), orientation);
} // end method

//-----------------------------------------------------------------------------

inline bool PhysicalLibraryCell::hasPolygonBoundaries() const {
	return data->clsPolygonBounds.getNumPoints() > 0;
} // end method 

//-----------------------------------------------------------------------------

inline const Polygon & PhysicalLibraryCell::getPolygonBoundaries() const {
	return data->clsPolygonBounds;
} // end method 

//-----------------------------------------------------------------------------

} // end  namespace 
