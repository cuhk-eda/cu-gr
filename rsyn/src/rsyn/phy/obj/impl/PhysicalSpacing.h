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

namespace Rsyn {

inline PhysicalLayer PhysicalSpacing::getLayer1() const {
	return data->clsLayer1;
} // end method 

// -----------------------------------------------------------------------------

inline PhysicalLayer PhysicalSpacing::getLayer2() const {
	return data->clsLayer2;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalSpacing::getDistance() const {
	return data->clsDistance;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 