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
 * File:   PhysicalDieData.h
 * Author: jucemar
 *
 * Created on 4 de Fevereiro de 2017, 08:40
 */

#ifndef PHYSICALDESIGN_PHYSICALDIEDATA_H
#define PHYSICALDESIGN_PHYSICALDIEDATA_H

namespace Rsyn {

class PhysicalDieData {
	friend class PhysicalDesign;
public:
	//! @brief	Rectangular PhysicalDie Bounds in DBU  units.
	Bounds clsBounds;
	//! @brief	It constructs the default Bounds(DBUxy(0,0), DBUxy(0,0)).
	PhysicalDieData() = default;
}; // end class 
} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALDIEDATA_H */

