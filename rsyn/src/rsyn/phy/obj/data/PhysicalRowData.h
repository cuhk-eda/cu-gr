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
 * File:   PhysicalRowData.h
 * Author: jucemar
 *
 * Created on 14 de Setembro de 2016, 22:26
 */

#ifndef PHYSICALDESIGN_PHYSICALROWDATA_H
#define PHYSICALDESIGN_PHYSICALROWDATA_H


namespace Rsyn {

class PhysicalRowData : public PhysicalObject {
public:
	Bounds clsBounds;
	PhysicalSite clsPhysicalSite;
	PhysicalOrientation clsSiteOrientation = Rsyn::PhysicalOrientation::ORIENTATION_INVALID;
	DBUxy clsOrigin;
	int clsNumSites[2] = {0, 0};
	DBUxy clsStep;
	std::string clsRowName;

	PhysicalRowData() {
		clsRowName = Rsyn::getPhysicalInvalidName();
	} // end constructor 

	// these methods are required in several parts of Physical Design.
	// Therefore, they are provided by physical row data
	// another solution was to create a variable to store each information.

	inline DBU getWidth() const {
		return clsNumSites[X] * clsStep[X];
	} // end method 

	inline DBU getHeight() const {
		return clsNumSites[Y] * clsStep[Y];
	} // end method 
}; // end class  

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALROWDATA_H */

