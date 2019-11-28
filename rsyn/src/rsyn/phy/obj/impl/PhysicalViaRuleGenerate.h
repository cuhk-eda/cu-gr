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
 * File:   PhysicalViaGenerate.h
 * Author: jucemar
 *
 * Created on November 12, 2018, 9:56 PM
 */

namespace Rsyn {

inline int PhysicalViaRuleGenerate::getRelativeIndex() const {
	return data->clsRelativeIndex;
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalViaRuleGenerate::isDefault() const {
	return data->clsIsDefault;
} // end method 

// -----------------------------------------------------------------------------

inline Rsyn::PhysicalLayer PhysicalViaRuleGenerate::getLayer(const Rsyn::PhysicalViaLayerType layer) const {
	return Rsyn::PhysicalLayer(data->clsLayers[layer]);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalViaRuleGenerate::getEnclosure1(const Rsyn::ViaLevel level) const {
	return data->clsEnclosure1[level];
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalViaRuleGenerate::getEnclosure2(const Rsyn::ViaLevel level) const {
	return data->clsEnclosure2[level];
} // end method 

// -----------------------------------------------------------------------------

inline Bounds PhysicalViaRuleGenerate::getCutBounds() const {
	return data->clsCutBounds;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalViaRuleGenerate::getCutSpacing(const Dimension dim) const {
	return data->clsCutSpacing[dim];
} // end method 

// -----------------------------------------------------------------------------

inline bool PhysicalViaRuleGenerate::hasCutResistance() const {
	return data->clsHasCutResistance;
} // end method 

// -----------------------------------------------------------------------------

inline float PhysicalViaRuleGenerate::getCutResistance() const {
	return data->clsCutResistance;
} // end method 

// -----------------------------------------------------------------------------


} // end namespace 