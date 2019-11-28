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
 * File:   PhysicalDescriptors.h
 * Author: jucemar
 *
 * Created on 6 de Julho de 2016, 15:29
 */


#ifndef LEFDESCRIPTORS_H
#define LEFDESCRIPTORS_H

#include <map>
#include <deque>

#include "rsyn/util/DoubleRectangle.h"
#include "rsyn/util/double2.h"

#include "lef5.8/lefrReader.hpp"

// NOTE: Guilherme Flach - 2016/11/04
// I did not change here from double to dbu because the data is load from
// lef as is and therefore it's in micron. The conversion to dbu is currently
// done in the physical layer initialization.

static const std::string INVALID_LEF_NAME = "*<INVALID_LEF_NAME>*";
static const std::string DEFAULT_PIN_DIRECTION = "INPUT";

// -----------------------------------------------------------------------------

//! Descriptor for LEF Polygon

class LefPolygonDscp {
public:
	std::vector<double2> clsPolygonPoints;
	LefPolygonDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

class LefPortGeometryDscp {
public:
	std::string clsMetalName = INVALID_LEF_NAME;
	std::vector<DoubleRectangle> clsBounds;
	std::vector<LefPolygonDscp> clsLefPolygonDscp;
}; // end class 

//! Descriptor for LEF Port

class LefPortDscp {
public:
	std::vector<LefPortGeometryDscp> clsLefPortGeoDscp;
	LefPortDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Pin

class LefPinDscp {
public:
	bool clsHasPort = false;
	std::string clsPinName = INVALID_LEF_NAME;
	std::string clsPinDirection = DEFAULT_PIN_DIRECTION;
	std::string clsPinUse = INVALID_LEF_NAME;
	DoubleRectangle clsBounds;
	std::vector<LefPortDscp> clsPorts;
	LefPinDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Obstruction

class LefObsDscp {
public:
	std::string clsMetalLayer = INVALID_LEF_NAME;
	std::vector<DoubleRectangle> clsBounds;
	LefObsDscp() = default;
};

// -----------------------------------------------------------------------------

//! Descriptor for LEF Macro

class LefMacroDscp {
public:
	lefiMacro* clsMacro = nullptr;
	std::vector<LefPinDscp> clsPins;
	std::vector<LefObsDscp> clsObs;
	LefMacroDscp() { clsMacro = new lefiMacro(); }
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Site

class LefSiteDscp {
public:
	std::string clsName = INVALID_LEF_NAME;
	double2 clsSize;
	bool clsHasClass = false;
	std::string clsSiteClass = INVALID_LEF_NAME;
	std::string clsSymmetry = INVALID_LEF_NAME;
	LefSiteDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Spacing

class LefSpacingDscp {
public:
	std::string clsLayer1 = INVALID_LEF_NAME;
	std::string clsLayer2 = INVALID_LEF_NAME;
	double clsDistance = 0.0;
	LefSpacingDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Via Layer. A via layer is composed by a layer and a set of rectangles.

class LefViaGeometryDscp {
public:
	DoubleRectangle clsBounds;
	int clsMask = 0;
	LefViaGeometryDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Via

class LefViaDscp {
public:
	bool clsIsDefault : 1;
	bool clsHasViaRule : 1;
	bool clsHasRowCol : 1;
	bool clsHasOrigin : 1;
	bool clsHasOffset : 1;
	bool clsHasResistance : 1;
	double clsXCutSize = 0;
	double clsYCutSize = 0;
	double clsXCutSpacing = 0;
	double clsYCutSpacing = 0;
	double clsXBottomEnclosure = 0;
	double clsYBottomEnclosure = 0;
	double clsXTopEnclosure = 0;
	double clsYTopEnclosure = 0;
	double clsXOrigin = 0;
	double clsYOrigin = 0;
	double clsXBottomOffset = 0;
	double clsYBottomOffset = 0;
	double clsXTopOffset = 0;
	double clsYTopOffset = 0;
	double clsCutResistance = 0.0;
	int clsNumCutRows = 0;
	int clsNumCutCols = 0;
	// map->first = layerName; map->second = list of geometry rects or polygons.
	std::map<std::string, std::deque<LefViaGeometryDscp>> clsGeometries;
	std::string clsName = INVALID_LEF_NAME;
	std::string clsViaRuleName = INVALID_LEF_NAME;
	std::string clsBottomLayer = INVALID_LEF_NAME;
	std::string clsCutLayer = INVALID_LEF_NAME;
	std::string clsTopLayer = INVALID_LEF_NAME;
	
	LefViaDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Layer of Via Rule and Via Rule Generate

class LefViaRuleLayerDscp {
public:
	bool clsHasDirection : 1;
	bool clsHasEnclosure : 1;
	bool clsHasWidth : 1;
	bool clsHasResistance : 1;
	bool clsHasOverhang : 1;
	bool clsHasMetalOverhang : 1;
	bool clsHasSpacing : 1;
	bool clsHasRect : 1;
	bool clsIsHorizontal : 1;
	bool clsIsVertical : 1;

	std::string clsName = INVALID_LEF_NAME;
	double clsEnclosure1 = 0.0;
	double clsEnclosure2 = 0.0;
	double clsCutResistance = 0.0;
	double clsMinWidth = 0.0;
	double clsMaxWidth = 0.0;
	double clsXSpacing  = 0.0;
	double clsYSpacing = 0.0;
	DoubleRectangle clsRect;
	LefViaRuleLayerDscp() {
		clsHasDirection = false;
		clsHasEnclosure = false;
		clsHasWidth = false;
		clsHasResistance = false;
		clsHasOverhang = false;
		clsHasMetalOverhang = false;
		clsHasSpacing = false;
		clsHasRect = false;
		clsIsHorizontal = false;
		clsIsVertical = false;
	} // end constructor
}; // end class 

//! Descriptor for LEF Via Rule and Via Rule Generate

class LefViaRuleDscp {
public:
	std::string clsName = INVALID_LEF_NAME;
	bool clsIsGenerate = false;
	bool clsIsDefault = false;
	std::vector<LefViaRuleLayerDscp> clsLayers;
	std::vector<std::string> clsVias;
	LefViaRuleDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF Units

class LefUnitsDscp {
public:
	bool clsHasTime = false;
	bool clsHasCapacitance = false;
	bool clsHasResitance = false;
	bool clsHasPower = false;
	bool clsHasCurrent = false;
	bool clsHasVoltage = false;
	bool clsHasDatabase = false;
	bool clsHasFrequency = false;

	int clsTime = 0;
	int clsCapacitance = 0;
	int clsResitance = 0;
	int clsPower = 0;
	int clsCurrent = 0;
	int clsVoltage = 0;
	int clsDatabase = 100; // default value at LEF/DEF reference
	int clsFrequency = 0;

	LefUnitsDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

//! Descriptor for LEF. The LEF elements are kept in the units defined at LefUnitsDscp.

class LefDscp {
public:
	int clsMajorVersion = 0;
	int clsMinorVersion = 0;
	std::string clsCaseSensitive = INVALID_LEF_NAME;
	std::string clsBusBitChars = INVALID_LEF_NAME;
	std::string clsDivideChar = INVALID_LEF_NAME;
	double clsManufactGrid = 0.0;
	LefUnitsDscp clsLefUnitsDscp;
	std::vector<LefSiteDscp> clsLefSiteDscps;
	std::vector<lefiLayer*> clsLefLayer;
	std::vector<LefMacroDscp> clsLefMacroDscps;
	std::vector<LefSpacingDscp> clsLefSpacingDscps;
	std::vector<LefViaDscp> clsLefViaDscps;
	std::deque<LefViaRuleDscp> clsLefViaRuleDscps;
	LefDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

#endif /* PHYSICALDESCRIPTORS_H */

