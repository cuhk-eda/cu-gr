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
 * File:   PhysicalSpecialWireData.h
 * Author: jucemar
 *
 * Created on 23 de Maio de 2017, 21:08
 */

#ifndef PHYSICALDESIGN_PHYSICALSPECIALWIREDATA_H
#define PHYSICALDESIGN_PHYSICALSPECIALWIREDATA_H

namespace Rsyn {

class PhysicalSpecialWireData : public PhysicalObject {
public:
	Rsyn::PhysicalLayer clsPhysicalLayer;
	DBU clsWireWidth;
	std::vector<PhysicalRoutingPoint> clsRoutingPoints;
	PhysicalSpecialWireData() = default;
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALSPECIALWIREDATA_H */

