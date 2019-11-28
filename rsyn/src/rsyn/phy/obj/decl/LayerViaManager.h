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
 * File:   LayerViaManager.h
 * Author: jucemar
 *
 * Created on February 3, 2018, 9:26 AM
 */

#ifndef LAYERVIAMANAGER_H
#define LAYERVIAMANAGER_H

namespace Rsyn {

class LayerViaManager : public Proxy<LayerViaManagerData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::LayerViaManager object with a pointer to Rsyn::LayerViaManagerData.

	LayerViaManager(LayerViaManagerData * data) : Proxy(data) {
	}
public:
	//! @brief Constructs a Rsyn::LayerViaManager object with a null pointer to Rsyn::LayerViaManagerData.

	LayerViaManager() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::LayerViaManager object with a null pointer to Rsyn::LayerViaManagerData.

	LayerViaManager(std::nullptr_t) : Proxy(nullptr) {
	}
	
	const std::vector<Rsyn::PhysicalVia> & allVias(Rsyn::PhysicalLayer layer) const;
	const std::vector<Rsyn::PhysicalVia> & allBottomVias(Rsyn::PhysicalLayer layer) const;
	const std::vector<Rsyn::PhysicalVia> & allTopVias(Rsyn::PhysicalLayer layer) const;
	bool hasVias(Rsyn::PhysicalLayer layer) const;
	bool hasBottomVias(Rsyn::PhysicalLayer layer) const;
	bool hasTopVias(Rsyn::PhysicalLayer layer) const;
	
}; // end class

} // end namespace 

#endif /* LAYERVIAMANAGER_H */

