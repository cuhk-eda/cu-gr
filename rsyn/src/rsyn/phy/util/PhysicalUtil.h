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

#ifndef PHYSICALDESIGN_PHYSICALUTIL_H
#define PHYSICALDESIGN_PHYSICALUTIL_H

#include <string>

#include "rsyn/core/RsynTypes.h"
#include "rsyn/phy/util/PhysicalTypes.h"

namespace Rsyn {

//! @file PhysicalUtil.h 
//! @brief This file aggregates conversion methods from/to Enum to/from string for Rsyn::PhysicalDesign.

//! Converts Layer type from string to enum, respectively.
Rsyn::PhysicalLayerType getPhysicalLayerType(const std::string &type);

//! Converts Layer type from enum to string, respectively.
std::string getPhysicalLayerType(const Rsyn::PhysicalLayerType type);

//! Converts Layer direction from string to enum, respectively.
Rsyn::PhysicalLayerDirection getPhysicalLayerDirection(const std::string &direction);

//! Converts Layer direction from enum to string, respectively.
std::string getPhysicalLayerDirection(const Rsyn::PhysicalLayerDirection direction);

//! Converts Orientation from string to enum, respectively.
Rsyn::PhysicalOrientation getPhysicalOrientation(const std::string &orientation);

//! Converts Orientation from enum to string, respectively.
std::string getPhysicalOrientation(const Rsyn::PhysicalOrientation orientation);

//! Converts Macro Class from string to enum, respectively.
Rsyn::PhysicalMacroClass getPhysicalMacroClass(const std::string & macroClass);

//! Converts Macro Class from enum to string, respectively.
std::string getPhysicalMacroClass(const Rsyn::PhysicalMacroClass macroClass);

//! Converts Symmetry from string to enum, respectively.
Rsyn::PhysicalSymmetry getPhysicalSymmetry(const std::string &rowSymmetry);

//! Converts Symmetry from enum to string, respectively.
std::string getPhysicalSymmetry(const Rsyn::PhysicalSymmetry rowSymmetry);

// ! @brief check if PhysicalSymmetry has symmetry to X.
bool isPhysicalSymmetryX(const Rsyn::PhysicalSymmetry symmetry);

// ! @brief check if PhysicalSymmetry has symmetry to Y.
bool isPhysicalSymmetryY(const Rsyn::PhysicalSymmetry symmetry);

// ! @brief check if PhysicalSymmetry has symmetry to XR90.
bool isPhysicalSymmetryR90(const Rsyn::PhysicalSymmetry symmetry);

//! Converts Site Class from string to enum, respectively.
Rsyn::PhysicalSiteClass getPhysicalSiteClass(const std::string & siteClass);

//! Converts Site Class from enum to string, respectively.
std::string getPhysicalSiteClass(const Rsyn::PhysicalSiteClass siteClass);

//! Converts Pin Direction from string to enum, respectively.
Rsyn::PhysicalPinDirection getPhysicalPinDirection(const std::string &direction);

//! Converts Pin Direction from enum to string, respectively.
std::string getPhysicalPinDirection(const Rsyn::PhysicalPinDirection pinDirection);

//! Converts Pin Geometry Class from string to enum, respectively.
Rsyn::PhysicalPinGeometryClass getPhysicalPinGeometryClass(const std::string & geometryClass);

//! Converts Pin Port Class from enum to string, respectively.
std::string getPhysicalPinGeometryClass(const Rsyn::PhysicalPinGeometryClass geometryClass);

//! Converts Region type from string to enum, respectively.
Rsyn::RegionType getPhysicalRegionType(const std::string & type);

//! Converts Region type from enum to string, respectively.
std::string getPhysicalRegionType(const Rsyn::RegionType type);

//! Converts Physical Design Mode type from string to enum, respectively.
Rsyn::PhysicalDesignMode getPhysicalDesignModeType(const std::string & type);

//! Converts Physical Design Mode type from enum to string, respectively.
std::string getPhysicalDesignModeType(const Rsyn::PhysicalDesignMode type);

//! Converts Physical Pin Use type from string to enum, respectively.
Rsyn::PhysicalPinUse getPhysicalPinUseType(const std::string & type);

//! Converts Physical Pin Use type from enum to string, respectively.
std::string getPhysicalPinUseType(const Rsyn::PhysicalPinUse type);

Rsyn::PhysicalTrackDirection getPhysicalTrackDirectionDEF(const std::string & type);

std::string getPhysicalTrackDirectionDEF(const Rsyn::PhysicalTrackDirection type);

std::string getPhysicalTrackDirection(const Rsyn::PhysicalTrackDirection type);


//! Returns a prefix key word to generated names.
std::string getPhysicalGeneratedNamePrefix();

//! Returns a prefix key word to invalid prefix.
std::string getPhysicalInvalidPrefix();

//! Returns a prefix key word to invalid name.
std::string getPhysicalInvalidName();

//! Returns a prefix key word to null name.
std::string getPhysicalNullName();



// *****************************************************************************
// *
// *         Overloading operator<<
// *
// *
// *****************************************************************************

// -----------------------------------------------------------------------------

//! Overloads operator << to Symmetry.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalSymmetry symmetry) {
	return out << getPhysicalSymmetry(symmetry);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Layer type.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalLayerType layerType) {
	return out << getPhysicalLayerType(layerType);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Layer Direction.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalLayerDirection direction) {
	return out << getPhysicalLayerDirection(direction);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Orientation.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalOrientation orientation) {
	return out << getPhysicalOrientation(orientation);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Macro Class.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalMacroClass macroClass) {
	return out << getPhysicalMacroClass(macroClass);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Site Class.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalSiteClass siteClass) {
	return out << getPhysicalSiteClass(siteClass);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Pin Direction.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalPinDirection pinDirection) {
	return out << getPhysicalPinDirection(pinDirection);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Pin Port Class.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalPinGeometryClass pinPortClass) {
	return out << getPhysicalPinGeometryClass(pinPortClass);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Region type.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::RegionType type) {
	return out << getPhysicalRegionType(type);
} // end operator overloading 

// -----------------------------------------------------------------------------


//! Overloads operator << to Physical Pin Use type.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalPinUse type) {
	return out << getPhysicalPinUseType(type);
} // end operator overloading 

// -----------------------------------------------------------------------------

//! Overloads operator << to Physical Track direction.

inline std::ostream& operator<<(std::ostream& out, const Rsyn::PhysicalTrackDirection type) {
	return out << getPhysicalTrackDirection(type);
} // end operator overloading 

// -----------------------------------------------------------------------------

} // end namespace

#endif /* PHYSICALDESIGN_PHYSICALUTIL_H */

