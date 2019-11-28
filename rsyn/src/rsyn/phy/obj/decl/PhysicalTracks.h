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
 * File:   PhysicalTracks.h
 * Author: jucemar
 *
 * Created on 24 de Maio de 2017, 21:51
 */

#ifndef PHYSICALDESIGN_PHYSICALTRACKS_H
#define PHYSICALDESIGN_PHYSICALTRACKS_H

namespace Rsyn {

class PhysicalTracks : public Proxy<PhysicalTracksData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
	RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;
protected:
	//! @brief Constructs a Rsyn::PhysicalTracks object with a pointer to Rsyn::PhysicalTracksData.

	PhysicalTracks(PhysicalTracksData * data) : Proxy(data) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalTracks object with a null pointer to Rsyn::PhysicalTracksData.

	PhysicalTracks() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalTracks object with a null pointer to Rsyn::PhysicalTracksData.

	PhysicalTracks(std::nullptr_t) : Proxy(nullptr) {
	}
	
	PhysicalTrackDirection getDirection() const;
	DBU getLocation() const;
	DBU getSpace() const;
	int getNumberOfTracks() const;
	std::size_t getNumberOfLayers() const;
	const std::vector<PhysicalLayer> & allLayers() const;
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALTRACKS_H */

