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
 * File:   ViaGeometryData.h
 * Author: jucemar
 *
 * Created on November 12, 2018, 5:44 PM
 */

#ifndef RSYN_DATABASE_VIAGEOMETRYDATA_H
#define RSYN_DATABASE_VIAGEOMETRYDATA_H

namespace Rsyn {
class ViaGeometryData : public PhysicalObject {
public:
	Bounds clsBounds;
	int clsMaskNumber = -1;
	// Polygon
}; // end class 

} // end namespace 
#endif /* RSYN_DATABASE_VIAGEOMETRYDATA_H */

