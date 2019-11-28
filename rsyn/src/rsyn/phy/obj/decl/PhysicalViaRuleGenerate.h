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
 * Created on November 12, 2018, 5:53 PM
 */

#ifndef RSYN_DATABASE_PHYSICALVIAGENERATE_H
#define RSYN_DATABASE_PHYSICALVIAGENERATE_H

namespace Rsyn {

class PhysicalViaRuleGenerate : public PhysicalViaRuleBase {
	friend class PhysicalDesign;
	friend class PhysicalViaRuleBase;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::PhysicalViaRuleGenerate object with a pointer to Rsyn::ViaRuleGenerateData.

	PhysicalViaRuleGenerate(ViaRuleData * data) : PhysicalViaRuleBase(data) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalViaRuleGenerate object with a null pointer to Rsyn::ViaRuleGenerateData.

	PhysicalViaRuleGenerate() : PhysicalViaRuleBase(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalViaRuleGenerate object with a null pointer to Rsyn::ViaRuleGenerateData.

	PhysicalViaRuleGenerate(std::nullptr_t) : PhysicalViaRuleBase(nullptr) {
	}

	//! @brief Returns the via rule relative index
	int getRelativeIndex() const;
	
	//! @brief Returns if the via rule generate can be used to regular routing
	bool isDefault() const;
	//! @brief Returns the via rule generate layer. It can be bottom, top or cut layer types 
	Rsyn::PhysicalLayer getLayer(const Rsyn::PhysicalViaLayerType layer) const;
	DBU getEnclosure1(const Rsyn::ViaLevel level) const;
	DBU getEnclosure2(const Rsyn::ViaLevel level) const;
	Bounds getCutBounds() const;
	DBU getCutSpacing(const Dimension dim) const;
	bool hasCutResistance() const;
	float getCutResistance() const;
private:
}; // end class

} // end namespace 

#endif /* PHYSICALVIAGENERATE_H */

