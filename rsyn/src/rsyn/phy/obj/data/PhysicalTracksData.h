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
 * File:   PhysicalTracksData.h
 * Author: jucemar
 *
 * Created on 24 de Maio de 2017, 21:47
 */

#ifndef PHYSICALDESIGN_PHYSICALTRACKSDATA_H
#define PHYSICALDESIGN_PHYSICALTRACKSDATA_H

namespace Rsyn {

class PhysicalTracksData : public PhysicalObject {
public:
	PhysicalTrackDirection clsDirection = PhysicalTrackDirection::INVALID_PHY_TRACK_DIRECTION;
	DBU clsLocation = 0;
	DBU clsSpace = 0;
	int clsNumTracks = 0;
	std::vector<PhysicalLayer> clsLayers;
}; // end class 
} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALTRACKSDATA_H */

