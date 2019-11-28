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
 
#ifndef ISPD18GUIDEDESCRIPTOR_H
#define ISPD18GUIDEDESCRIPTOR_H

#include <deque>
#include "rsyn/util/Bounds.h"

class GuideLayerDscp {
public:
	Bounds clsLayerGuide;
	std::string clsLayer = "";
	GuideLayerDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

class GuideNetDscp {
public:
	std::string clsNetName = "";
	std::deque<GuideLayerDscp> clsLayerDscps;
	GuideNetDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

class GuideDscp {
public:
	std::deque<GuideNetDscp> clsNetGuides;
	GuideDscp() = default;
}; // end class 

// -----------------------------------------------------------------------------

#endif /* ISPD18GUIDEDESCRIPTOR_H */

