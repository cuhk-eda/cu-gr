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
 * File:   PhysicalObject.h
 * Author: jucemar
 *
 * Created on 12 de Setembro de 2016, 19:59
 */

#ifndef PHYSICALDESIGN_PHYSICALOBJECT_H
#define PHYSICALDESIGN_PHYSICALOBJECT_H


namespace Rsyn {

class PhysicalObject {
public:
	//! @brief Unique identifier of the Physical Objects. 
	//! @details It is used to avoid compare pointer when sorting physical objects. 
	//! The id variable is protected from users access outside of Physical classes. 
	PhysicalIndex id = INVALID_PHYSICAL_INDEX;

	PhysicalObject() = default;
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALOBJECT_H */

