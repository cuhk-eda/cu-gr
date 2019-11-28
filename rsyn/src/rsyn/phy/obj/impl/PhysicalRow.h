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
 * File:   PhysicalRow.h
 * Author: jucemar
 *
 * Created on 18 de Setembro de 2016, 09:27
 */
namespace Rsyn {

inline std::string PhysicalRow::getName() const {
	return data->clsRowName;
} // end method 

// -----------------------------------------------------------------------------

inline std::string PhysicalRow::getSiteName() const {
	return data->clsPhysicalSite.getName();
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalRow::getWidth() const {
	return data->clsNumSites[X] * data->clsStep[X];
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalRow::getHeight() const {
	return data->clsNumSites[Y] * data->clsStep[Y];
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalRow::getOrigin(const Dimension dim) const {
	return data->clsOrigin[dim];
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalRow::getOrigin() const {
	return data->clsOrigin;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalRow::getStep(const Dimension dim) const {
	return data->clsStep[dim];
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalRow::getStep() const {
	return data->clsStep;
} // end method 

// -----------------------------------------------------------------------------

inline int PhysicalRow::getNumSites(const Dimension dim) const  {
	return data->clsNumSites[dim];
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalSite PhysicalRow::getPhysicalSite() const {
	return data->clsPhysicalSite;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalRow::getSiteWidth() const {
	return data->clsPhysicalSite.getWidth();
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalRow::getSiteHeight() const {
	return data->clsPhysicalSite.getHeight();
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalRow::getCoordinate(const Boundary bound) const {
	return data->clsBounds[bound];
} // end method 

// ----------------------------------------------------------------------------- 

inline DBU PhysicalRow::getCoordinate(const Boundary bound, const Dimension dim) const {
	return data->clsBounds[bound][dim];
} // end method 

// -----------------------------------------------------------------------------

inline const Bounds &PhysicalRow::getBounds() const {
	return data->clsBounds;
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalOrientation PhysicalRow::getSiteOrientation() const {
	return data->clsSiteOrientation;
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalSymmetry PhysicalRow::getSymmetry() const {
	return data->clsPhysicalSite ? data->clsPhysicalSite.getSymmetry() : Rsyn::PhysicalSymmetry::SYMMETRY_INVALID;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 