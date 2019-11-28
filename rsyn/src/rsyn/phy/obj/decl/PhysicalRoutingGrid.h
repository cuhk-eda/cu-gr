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
 * File:   PhysicalRoutingGrid.h
 * Author: jucemar
 *
 * Created on February 2, 2018, 9:01 PM
 */

#ifndef RSYN_PHYSICAL_ROUTING_GRID_H
#define RSYN_PHYSICAL_ROUTING_GRID_H

namespace Rsyn {

class PhysicalRoutingGrid : public Proxy<PhysicalRoutingGridData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::PhysicalRoutingGrid object with a pointer to Rsyn::PhysicalRoutingGridData.

	PhysicalRoutingGrid(PhysicalRoutingGridData * data) : Proxy(data) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalRoutingGrid object with a null pointer to Rsyn::PhysicalRoutingGridData.

	PhysicalRoutingGrid() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalRoutingGrid object with a null pointer to Rsyn::PhysicalRoutingGridData.

	PhysicalRoutingGrid(std::nullptr_t) : Proxy(nullptr) {
	}
	
	Rsyn::PhysicalLayer getLayer() const;
	const std::vector<Rsyn::PhysicalTracks> & allTracks() const;
	
	const Bounds & getBounds() const;
	DBUxy getPosition() const;
	DBU getPosition(const Dimension dim) const;
	DBUxy getSpacing() const;
	DBU getSpacing(const Dimension dim) const;
	int getNumTracks(const Dimension dim) const;
	int getNumRows() const;
	int getNumCols() const;
	int getNumTracks() const;
	int getRow(const DBU posY, const RoundingStrategy roudingStrategy = ROUND_NEAREST, const bool clamp = false) const;
	int getCol(const DBU posX, const RoundingStrategy roudingStrategy = ROUND_NEAREST, const bool clamp = false) const;
	DBUxy getPosition(const int col, const int row) const;
	DBUxy getSnappedPosition(const DBUxy pos, const RoundingStrategy roudingStrategy = ROUND_NEAREST, const bool clamp = false) const;
	DBU getRowPosition(const int row) const;
	DBU getRowMaxPosition() const;
	DBU getColPosition(const int col) const;
	DBU getColMaxPosition() const;
	DBUxy getTrackMaxPosition() const;
	DBU getTrackMaxPosition(const Dimension dim) const;
	Rsyn::PhysicalRoutingGrid getBottomRoutingGrid() const;
	Rsyn::PhysicalRoutingGrid getTopRoutingGrid() const;
	bool hasBottomRoutingGrid() const;
	bool hasTopRoutingGrid() const;

	DBUxy getGridMinPosition() const;
	DBUxy getGridMaxPosition() const;
	DBUxy getTrackMinPosition(const PhysicalLayerDirection dir, const int index) const;
	DBUxy getTrackMaxPosition(const PhysicalLayerDirection dir, const int index) const;
	
}; // end class 

} // end namespace 

#endif /* PHYSICALROUTINGGRID_H */

