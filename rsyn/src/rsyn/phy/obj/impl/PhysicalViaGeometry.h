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
 * File:   PhysicalViaGeometry.h
 * Author: jucemar
 *
 * Created on November 12, 2018, 10:14 PM
 */

namespace Rsyn {

inline const Bounds & PhysicalViaGeometry::getBounds() const {
	return data->clsBounds;
} // end method 

// -----------------------------------------------------------------------------

inline int PhysicalViaGeometry::getMaskNumber() const {
	return data->clsMaskNumber;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 

