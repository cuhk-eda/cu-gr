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
 * File:   PhysicalModule.h
 * Author: jucemar
 *
 * Created on 4 de Outubro de 2016, 20:34
 */

#ifndef PHYSICALDESIGN_PHYSICALMODULE_H
#define PHYSICALDESIGN_PHYSICALMODULE_H

namespace Rsyn {

class PhysicalModule : public PhysicalInstance {
	friend class PhysicalDesign;
protected:
	//! @brief Constructs a Rsyn::PhysicalModule object with a pointer to Rsyn::PhysicalInstanceData.
	PhysicalModule(PhysicalInstanceData * data) : PhysicalInstance(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalModule object with a null pointer to Rsyn::PhysicalInstanceData.
	PhysicalModule() : PhysicalInstance(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalModule object with a null pointer to Rsyn::PhysicalInstanceData.
	PhysicalModule(std::nullptr_t) : PhysicalInstance(nullptr) {}
}; // end class

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALMODULE_H */

