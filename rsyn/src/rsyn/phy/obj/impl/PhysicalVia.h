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
 * File:   PhysicalVia.h
 * Author: jucemar
 *
 * Created on 14 de Maio de 2017, 15:25
 */

namespace Rsyn {

inline const std::string & PhysicalVia::getName() const {
	return data->clsName;
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalLayer PhysicalVia::getTopLayer() const {
	return PhysicalLayer(data->clsLayers[TOP_VIA_LAYER]);
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalLayer PhysicalVia::getCutLayer() const {
	return PhysicalLayer(data->clsLayers[CUT_VIA_LAYER]);
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalLayer PhysicalVia::getBottomLayer() const {
	return PhysicalLayer(data->clsLayers[BOTTOM_VIA_LAYER]);
} // end method

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalLayer PhysicalVia::getLayer(const PhysicalViaLayerType type) const {
	return PhysicalLayer(data->clsLayers[type]);
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::isViaDesign() const {
	return data->clsIsViaDesign;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::isViaRule() const {
	return data->clsType == VIA_RULE_TYPE;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::isViaGeometry() const {
	return data->clsType == VIA_GEOMETRY_TYPE;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::hasViaRule() const {
	return data->clsHasViaRule;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::hasRowCol() const {
	return data->clsHasRowCol;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::hasOrigin() const {
	return data->clsHasOrigin;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::hasOffset() const {
	return data->clsHasOffset;
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalVia::hasPattern() const {
	return data->clsHasPattern;
} // end method

// -----------------------------------------------------------------------------

inline ViaType PhysicalVia::getViaType() const {
	return data->clsType;
} // end method

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalViaRuleBase PhysicalVia::getViaRule() const {
	return PhysicalViaRuleBase(data->clsViaRuleData);
} // end method

// -----------------------------------------------------------------------------

inline DBU PhysicalVia::getCutSize(const Dimension dim) const {
	return data->clsCutSize[dim];
} // end method

// -----------------------------------------------------------------------------

inline DBU PhysicalVia::getSpacing(const Dimension dim) const {
	return data->clsSpacing[dim];
} // end method

// -----------------------------------------------------------------------------

inline DBU PhysicalVia::getEnclosure(const ViaLevel level, const Dimension dim) const {
	return data->clsEnclosure[level][dim];
} // end method

// -----------------------------------------------------------------------------

inline DBU PhysicalVia::getOrigin(const Dimension dim) const {
	return data->clsOrigin[dim];
} // end method

// -----------------------------------------------------------------------------

inline DBU PhysicalVia::getOffset(const ViaLevel level, const Dimension dim) const {
	return data->clsOffset[level][dim];
} // end method

// -----------------------------------------------------------------------------

inline int PhysicalVia::getNumRows() const {
	return data->clsNumRows;
} // end method

// -----------------------------------------------------------------------------

inline int PhysicalVia::getNumCols() const {
	return data->clsNumCols;
} // end method

// -----------------------------------------------------------------------------

inline const std::string & PhysicalVia::getPattern() const {
	return data->clsPattern;
} // end method

// -----------------------------------------------------------------------------

inline const std::vector<Rsyn::PhysicalViaGeometry> & PhysicalVia::allBottomGeometries() const {
	return allGeometries(BOTTOM_VIA_LAYER);
} // end method

// -----------------------------------------------------------------------------

inline const std::vector<Rsyn::PhysicalViaGeometry> & PhysicalVia::allCutGeometries() const {
	return allGeometries(CUT_VIA_LAYER);
} // end method

// -----------------------------------------------------------------------------

inline const std::vector<Rsyn::PhysicalViaGeometry> & PhysicalVia::allTopGeometries() const {
	return allGeometries(TOP_VIA_LAYER);
} // end method

// -----------------------------------------------------------------------------

inline const std::vector<Rsyn::PhysicalViaGeometry> & PhysicalVia::allGeometries(const PhysicalViaLayerType layer) const {
	return data->clsViaGeometries[layer];
} // end method

// -----------------------------------------------------------------------------

} // end namespace 

