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
 * File:   PhysicalLayerViaManagerData.h
 * Author: jucemar
 *
 * Created on February 3, 2018, 9:17 AM
 */

#ifndef PHYSICALLAYERVIAMANAGERDATA_H
#define PHYSICALLAYERVIAMANAGERDATA_H

namespace Rsyn {

class LayerViaManagerData : public PhysicalObject {
public:
	std::map<Rsyn::PhysicalLayer, std::vector<Rsyn::PhysicalVia>>  clsVias;
	std::map<Rsyn::PhysicalLayer, std::vector<Rsyn::PhysicalVia>>  clsBottomVias;
	std::map<Rsyn::PhysicalLayer, std::vector<Rsyn::PhysicalVia>>  clsTopVias;
	LayerViaManagerData() = default;
}; // end class 

} // end namespace 


#endif /* PHYSICALLAYERVIAMANAGERDATA_H */

