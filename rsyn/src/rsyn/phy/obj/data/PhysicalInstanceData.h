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
 * File:   PhysicalCellData.h
 * Author: jucemar
 *
 * Created on 12 de Setembro de 2016, 21:01
 */

#ifndef PHYSICALDESIGN_PHYSICALINSTANCEDATA_H
#define PHYSICALDESIGN_PHYSICALINSTANCEDATA_H
namespace Rsyn {

class PhysicalInstanceData {
	friend class PhysicalDesign;
public:
	bool clsBlock : 1;
	bool clsPlaced : 1;
	bool clsPort : 1;
	bool clsHasLayerBounds : 1; //  the bounds of a cell is defined by one of the layers. 

	DBUxy clsInitialPos; // Initial position from global placement when in detailed placement has maximal displacement
	PhysicalLayer clsPortLayer;
	Rsyn::Instance clsInstance;

	PhysicalInstanceData() {
		clsBlock = false;
		clsPlaced = false;
		clsPort = false;
		clsHasLayerBounds = false;
	} // end constructor 
}; // end class

} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALINSTANCEDATA_H */

