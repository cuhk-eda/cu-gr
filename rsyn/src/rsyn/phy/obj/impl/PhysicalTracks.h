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
 * Created on 24 de Maio de 2017, 21:53
 */

namespace Rsyn {

inline PhysicalTrackDirection PhysicalTracks::getDirection() const {
	return data->clsDirection;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalTracks::getLocation() const {
	return data->clsLocation;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalTracks::getSpace() const {
	return data->clsSpace;
} // end method 

// -----------------------------------------------------------------------------

inline int PhysicalTracks::getNumberOfTracks() const {
	return data->clsNumTracks;
} // end method 

// -----------------------------------------------------------------------------

inline std::size_t PhysicalTracks::getNumberOfLayers() const {
	return data->clsLayers.size();
} // end method 

// -----------------------------------------------------------------------------

inline const std::vector<PhysicalLayer> & PhysicalTracks::allLayers() const {
	return data->clsLayers;
} // end method 

// -----------------------------------------------------------------------------
} // end namespace 

