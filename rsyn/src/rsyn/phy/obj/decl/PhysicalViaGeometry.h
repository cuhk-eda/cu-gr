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
 * File:   PhysicalViaGeometry.h
 * Author: jucemar
 *
 * Created on November 12, 2018, 5:52 PM
 */

#ifndef RSYN_DATABASE_PHYSICALVIAGEOMETRY_H
#define RSYN_DATABASE_PHYSICALVIAGEOMETRY_H

namespace Rsyn {

class PhysicalViaGeometry : public Proxy<ViaGeometryData> {
	friend class PhysicalDesign;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::PhysicalViaGeometry object with a pointer to Rsyn::ViaGeometryData.

	PhysicalViaGeometry(ViaGeometryData * data) : Proxy(data) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalViaGeometry object with a null pointer to Rsyn::ViaGeometryData.

	PhysicalViaGeometry() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalViaGeometry object with a null pointer to Rsyn::ViaGeometryData.

	PhysicalViaGeometry(std::nullptr_t) : Proxy(nullptr) {
	}
	
	const Bounds & getBounds() const;
	int getMaskNumber() const;
	
}; // end class

} // end namespace

#endif /* RSYN_DATABASE_PHYSICALVIAGEOMETRY_H */

