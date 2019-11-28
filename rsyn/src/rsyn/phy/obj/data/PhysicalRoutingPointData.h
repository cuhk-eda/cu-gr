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
 * File:   PhysicalRoutingPointData.h
 * Author: jucemar
 *
 * Created on 23 de Maio de 2017, 20:27
 */

#ifndef PHYSICALDESIGN_PHYSICALROUTINGPOINTDATA_H
#define PHYSICALDESIGN_PHYSICALROUTINGPOINTDATA_H

namespace Rsyn {

class PhysicalRoutingPointData : public PhysicalObject {
public:
	PhysicalVia clsVia;
	DBUxy clsPos;
	Bounds clsRectangle;
	DBU clsExtension = -1;
	PhysicalOrientation clsOrientation = ORIENTATION_N; // default routing point orientation
	bool clsHasRectangle : 1;
	bool clsHasMask : 1;
	bool clsHasVirtual : 1;
	PhysicalRoutingPointData() {
		clsHasRectangle = false;
		clsHasMask = false;
		clsHasVirtual = false;
	} // end constructor 
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALROUTINGPOINTDATA_H */

