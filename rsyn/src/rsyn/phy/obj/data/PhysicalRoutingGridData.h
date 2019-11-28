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
 * File:   PhysicalRoutingGridData.h
 * Author: jucemar
 *
 * Created on February 2, 2018, 9:00 PM
 */

#ifndef PHYSICALROUTINGGRIDDATA_H
#define PHYSICALROUTINGGRIDDATA_H

namespace Rsyn {

class PhysicalRoutingGridData : public PhysicalObject {
	friend class PhysicalDesign;
public:
	std::vector<Rsyn::PhysicalTracks> clsTracks;
	Rsyn::PhysicalLayer clsLayer;
	DBUxy clsSpacing;
	int clsNumTracks[2];
	Bounds clsBounds;
	Rsyn::PhysicalRoutingGrid clsBottomRoutingGrid;
	Rsyn::PhysicalRoutingGrid clsTopRoutingGrid;
	PhysicalRoutingGridData() = default;
}; // end class

} // end namespace

#endif /* PHYSICALROUTINGGRIDDATA_H */

