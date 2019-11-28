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
 * File:   PhysicalNetData.h
 * Author: jucemar
 *
 * Created on 15 de Setembro de 2016, 19:15
 */

#ifndef RSYN_PHYSICAL_NET_DATA_H
#define RSYN_PHYSICAL_NET_DATA_H

namespace Rsyn {

class PhysicalNetData {
public:
	Bounds clsBounds;
	Rsyn::Net clsNet;
	Rsyn::Pin clsBoundPins[2][2];
	PhysicalRouting clsRouting;

	PhysicalNetData() {
		clsBoundPins[LOWER][X] = nullptr;
		clsBoundPins[LOWER][Y] = nullptr;
		clsBoundPins[UPPER][X] = nullptr;
		clsBoundPins[UPPER][Y] = nullptr;
	} // end constructor 
}; // end class

} // end class 

#endif

