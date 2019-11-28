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

#include "PhysicalUtil.h"

namespace Rsyn {

static const std::string NULL_PHY_NAME = "*<NULL_PHY_NAME>*";
static const std::string GEN_NAME = "*<GEN_NAME>*_";
static const std::string INVALID_NAME = "*<INVALID_NAME>*";

Rsyn::PhysicalLayerType getPhysicalLayerType(const std::string & type) {
	if (type.compare("ROUTING") == 0) return ROUTING;
	if (type.compare("CUT") == 0) return CUT;
	if (type.compare("OVERLAP") == 0) return OVERLAP;
	if (type.compare("MASTERSLICE") == 0) return MASTERSLICE;
	if (type.compare("IMPLANT") == 0) return IMPLANT;
	return INVALID_PHY_LAYER_TYPE;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalLayerType(const Rsyn::PhysicalLayerType type) {
	switch (type) {
		case ROUTING: return "ROUTING";
		case CUT: return "CUT";
		case OVERLAP: return "OVERLAP";
		case MASTERSLICE: return "MASTERSLICE";
		case IMPLANT: return "IMPLANT";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalLayerDirection getPhysicalLayerDirection(const std::string & direction) {
	if (direction.compare("HORIZONTAL") == 0) return HORIZONTAL;
	if (direction.compare("VERTICAL") == 0) return VERTICAL;
	return UNKNOWN_PREFERRED_DIRECTION;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalLayerDirection(const PhysicalLayerDirection direction) {
	switch (direction) {
		case HORIZONTAL: return "HORIZONTAL";
		case VERTICAL: return "VERTICAL";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalOrientation getPhysicalOrientation(const std::string &orientation) {
	if (orientation.compare("N") == 0) return ORIENTATION_N;
	if (orientation.compare("S") == 0) return ORIENTATION_S;
	if (orientation.compare("E") == 0) return ORIENTATION_E;
	if (orientation.compare("W") == 0) return ORIENTATION_W;

	if (orientation.compare("FN") == 0) return ORIENTATION_FN;
	if (orientation.compare("FS") == 0) return ORIENTATION_FS;
	if (orientation.compare("FE") == 0) return ORIENTATION_FE;
	if (orientation.compare("FW") == 0) return ORIENTATION_FW;

	return ORIENTATION_INVALID;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalOrientation(const Rsyn::PhysicalOrientation orientation) {
	switch (orientation) {
		case ORIENTATION_N: return "N";
		case ORIENTATION_S: return "S";
		case ORIENTATION_E: return "E";
		case ORIENTATION_W: return "W";

		case ORIENTATION_FN: return "FN";
		case ORIENTATION_FS: return "FS";
		case ORIENTATION_FE: return "FE";
		case ORIENTATION_FW: return "FW";

		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalMacroClass getPhysicalMacroClass(const std::string & macroClass) {
	if (macroClass.compare("COVER") == 0) return MACRO_COVER;
	if (macroClass.compare("RING") == 0) return MACRO_RING;
	if (macroClass.compare("BLOCK") == 0) return MACRO_BLOCK;
	if (macroClass.compare("PAD") == 0) return MACRO_PAD;
	if (macroClass.compare("CORE") == 0) return MACRO_CORE;
	if (macroClass.compare("ENDCAP") == 0) return MACRO_ENDCAP;

	return MACRO_INVALID_CLASS;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalMacroClass(const Rsyn::PhysicalMacroClass macroClass) {
	switch (macroClass) {
		case MACRO_COVER: return "COVER";
		case MACRO_RING: return "RING";
		case MACRO_BLOCK: return "BLOCK";
		case MACRO_PAD: return "PAD";
		case MACRO_CORE: return "CORE";
		case MACRO_ENDCAP: return "ENDCAP";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalSymmetry getPhysicalSymmetry(const std::string &rowSymmetry) {
	bool hasX = std::string::npos != rowSymmetry.find("X");
	bool hasY = std::string::npos != rowSymmetry.find("Y");
	bool hasR90 = std::string::npos != rowSymmetry.find("R90");


	if (hasX && hasY && hasR90) return SYMMETRY_XYR90;

	if (hasY && hasR90) return SYMMETRY_YR90;
	if (hasX && hasR90) return SYMMETRY_XR90;
	if (hasX && hasY) return SYMMETRY_XY;

	if (hasR90) return SYMMETRY_R90;
	if (hasY) return SYMMETRY_Y;
	if (hasX) return SYMMETRY_X;

	return SYMMETRY_INVALID;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalSymmetry(const Rsyn::PhysicalSymmetry rowSymmetry) {
	switch (rowSymmetry) {
		case SYMMETRY_X: return "X";
			break;
		case SYMMETRY_Y: return "Y";
			break;
		case SYMMETRY_R90: return "R90";
			break;
		case SYMMETRY_XY: return "X Y";
			break;
		case SYMMETRY_XR90: return "X R90";
			break;
		case SYMMETRY_YR90: return "Y R90";
			break;
		case SYMMETRY_XYR90: return "X Y R90";
			break;
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

bool isPhysicalSymmetryX(const Rsyn::PhysicalSymmetry symmetry) {
	return symmetry == SYMMETRY_X || symmetry == SYMMETRY_XY || 
		symmetry == SYMMETRY_XR90 || SYMMETRY_XYR90;
} // end method 

// -----------------------------------------------------------------------------

bool isPhysicalSymmetryY(const Rsyn::PhysicalSymmetry symmetry) {
	return symmetry == SYMMETRY_Y || symmetry == SYMMETRY_XY || 
		symmetry == SYMMETRY_YR90 || SYMMETRY_XYR90;
} // end method 

// -----------------------------------------------------------------------------

bool isPhysicalSymmetryR90(const Rsyn::PhysicalSymmetry symmetry) {
	return symmetry == SYMMETRY_R90 || symmetry == SYMMETRY_XR90 || 
		symmetry == SYMMETRY_YR90 || SYMMETRY_XYR90;
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalSiteClass getPhysicalSiteClass(const std::string & siteClass) {
	if (siteClass.compare("CORE") == 0) return PhysicalSiteClass::CORE;
	if (siteClass.compare("PAD") == 0) return PhysicalSiteClass::PAD;

	return PhysicalSiteClass::INVALID_SITECLASS;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalSiteClass(const Rsyn::PhysicalSiteClass siteClass) {
	switch (siteClass) {
		case CORE: return "CORE";
			break;
		case PAD: return "PAD";
			break;
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

Rsyn::PhysicalPinDirection getPhysicalPinDirection(const std::string &direction) {
	if (direction.compare("INPUT") == 0) return PIN_INPUT;
	if (direction.compare("OUTPUT") == 0) return PIN_OUTPUT;

	return PIN_INVALID_DIRECTION;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalPinDirection(const Rsyn::PhysicalPinDirection pinDirection) {
	switch (pinDirection) {
		case PIN_INPUT: return "INPUT";
		case PIN_OUTPUT: return "OUTPUT";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

Rsyn::PhysicalPinGeometryClass getPhysicalPinGeometryClass(const std::string & geometryClass) {
	if (geometryClass.compare("NONE") == 0) return PINGEOMETRYCLASS_NONE;
	if (geometryClass.compare("CORE") == 0) return PINGEOMETRYCLASS_CORE;
	if (geometryClass.compare("BUMP") == 0) return PINGEOMETRYCLASS_BUMP;

	return PINGEOMETRYCLASS_INVALID;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalPinGeometryClass(const Rsyn::PhysicalPinGeometryClass geometryClass) {
	switch (geometryClass) {
		case PINGEOMETRYCLASS_NONE: return "NONE";
		case PINGEOMETRYCLASS_CORE: return "CORE";
		case PINGEOMETRYCLASS_BUMP: return "BUMP";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

Rsyn::RegionType getPhysicalRegionType(const std::string & type) {
	if (type.compare("FENCE") == 0) return RegionType::FENCE;
	if (type.compare("GUIDE") == 0) return RegionType::GUIDE;
	return RegionType::INVALID;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalRegionType(const Rsyn::RegionType type) {
	switch (type) {
		case RegionType::FENCE: return "FENCE";
		case RegionType::GUIDE: return "GUIDE";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalDesignMode getPhysicalDesignModeType(const std::string & type) {
	if (type.compare("ALL") == 0) return PhysicalDesignMode::ALL;
	if (type.compare("CTS") == 0) return PhysicalDesignMode::CTS;
	if (type.compare("FLOORPLANNING") == 0) return PhysicalDesignMode::FLOORPLANNING;
	if (type.compare("PLACEMENT") == 0) return PhysicalDesignMode::PLACEMENT;
	if (type.compare("ROUTING") == 0) return PhysicalDesignMode::ROUTING;
	return PhysicalDesignMode::INVALID;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalDesignModeType(const Rsyn::PhysicalDesignMode type) {
	switch (type) {
		case PhysicalDesignMode::ALL: return "ALL";
		case PhysicalDesignMode::CTS: return "CTS";
		case PhysicalDesignMode::FLOORPLANNING: return "FLOORPLANNING";
		case PhysicalDesignMode::PLACEMENT: return "PLACEMENT";
		case PhysicalDesignMode::ROUTING: return "ROUTING";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

//! Converts Physical Pin Use type from string to enum, respectively.

Rsyn::PhysicalPinUse getPhysicalPinUseType(const std::string & type) {
	if (type.compare("SIGNAL") == 0) return PhysicalPinUse::PIN_USE_SIGNAL;
	if (type.compare("ANALOG") == 0) return PhysicalPinUse::PIN_USE_ANALOG;
	if (type.compare("POWER") == 0) return PhysicalPinUse::PIN_USE_POWER;
	if (type.compare("GROUND") == 0) return PhysicalPinUse::PIN_USE_GROUND;
	if (type.compare("CLOCK") == 0) return PhysicalPinUse::PIN_USE_CLOCK;
	return PhysicalPinUse::PIN_INVALID_USE;
} // end method 

// -----------------------------------------------------------------------------

//! Converts Physical Pin Use type from enum to string, respectively.

std::string getPhysicalPinUseType(const Rsyn::PhysicalPinUse type) {
	switch (type) {
		case PhysicalPinUse::PIN_USE_SIGNAL: return "SIGNAL";
		case PhysicalPinUse::PIN_USE_ANALOG: return "ANALOG";
		case PhysicalPinUse::PIN_USE_POWER: return "POWER";
		case PhysicalPinUse::PIN_USE_GROUND: return "GROUND";
		case PhysicalPinUse::PIN_USE_CLOCK: return "CLOCK";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

Rsyn::PhysicalTrackDirection getPhysicalTrackDirectionDEF(const std::string & type) {
	if (type.compare("X") == 0) return Rsyn::PhysicalTrackDirection::TRACK_VERTICAL;
	if (type.compare("Y") == 0) return Rsyn::PhysicalTrackDirection::TRACK_HORIZONTAL;
	return Rsyn::PhysicalTrackDirection::INVALID_PHY_TRACK_DIRECTION;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalTrackDirectionDEF(const Rsyn::PhysicalTrackDirection type) {
	switch (type) {
		case PhysicalTrackDirection::TRACK_VERTICAL: return "X";
		case PhysicalTrackDirection::TRACK_HORIZONTAL: return "Y";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalTrackDirection(const Rsyn::PhysicalTrackDirection type) {
	switch (type) {
		case PhysicalTrackDirection::TRACK_VERTICAL: return "HORIZONTAL";
		case PhysicalTrackDirection::TRACK_HORIZONTAL: return "VERTICAL";
		default: return Rsyn::getPhysicalInvalidName();
	} // end switch 
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalGeneratedNamePrefix() {
	return GEN_NAME;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalInvalidPrefix() {
	return INVALID_NAME;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalInvalidName() {
	return INVALID_NAME;
} // end method 

// -----------------------------------------------------------------------------

std::string getPhysicalNullName() {
	return NULL_PHY_NAME;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 

