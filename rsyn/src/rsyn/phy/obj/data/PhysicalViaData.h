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
 * File:   PhysicalViaData.h
 * Author: jucemar
 *
 * Created on 14 de Maio de 2017, 14:49
 */

#ifndef PHYSICALDESIGN_PHYSICALVIADATA_H
#define PHYSICALDESIGN_PHYSICALVIADATA_H

namespace Rsyn {

class PhysicalViaData : public PhysicalObject {
public:
	bool clsIsDefault : 1;
	bool clsIsViaDesign : 1;
	bool clsHasViaRule : 1;
	bool clsHasRowCol : 1;
	bool clsHasOrigin : 1;
	bool clsHasOffset : 1;
	bool clsHasPattern : 1;
	bool clsHasCutResistance : 1;
	ViaType clsType = INVALID_VIA_TYPE;
	Rsyn::ViaRuleData * clsViaRuleData = nullptr;
	DBU clsCutSize [2] = {0, 0}; // xCutSize, yCutSize
	DBU clsSpacing [2] = {0, 0}; // xSpacing, ySpacing
	DBU clsEnclosure[NUM_VIA_LEVELS][2]; // bottomXEnclosure, bottomYEnclosure, topXEnclosure, topYEnclosure
	DBU clsOrigin[2] = {0, 0}; // xOrigin, yOrigin
	DBU clsOffset[NUM_VIA_LEVELS][2]; // bottomXOffset, bottomYOffset, topXOffset, topYOffset
	int clsNumRows = 0;
	int clsNumCols = 0;
    std::string clsPattern = "";
	float clsCutResistance = 0.0;
	Rsyn::PhysicalLayerData *clsLayers[NUM_VIA_LAYERS]; // bottom, cut, top
	std::vector<Rsyn::PhysicalViaGeometry>  clsViaGeometries [NUM_VIA_LAYERS]; // bottom, cut, top
	std::string clsName;
	
	PhysicalViaData() {
		clsIsDefault = false;
		clsIsViaDesign = false;
		clsHasViaRule = false;
		clsHasRowCol = false;
		clsHasOrigin = false;
		clsHasOffset = false;
		clsHasPattern = false;
		clsHasCutResistance = false;
	} // end constructor 
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALVIADATA_H */

