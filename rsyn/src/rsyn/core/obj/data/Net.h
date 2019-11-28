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
 
namespace Rsyn {

struct NetTagData {
	TristateFlag ideal;
	NetTypeTag type;

	NetTagData() :
		type(NET_TYPE_TAG_NOT_SPECIFIED) {
	} // end constructor
}; // end struct

// -----------------------------------------------------------------------------

struct NetData : ObjectData {
	// Index in parent module.
	Index mid;

	// User tags.
	NetTagData tag;
	
	// Using a vector for fast traverse, but slow insertion and removal.
	std::vector<Pin> pins;
	
	// Driver. If multiple-drivers, store one of them without any assumptions.
	Pin driver;
	
	// Parent
	Module parent;
	
	std::array<int, NUM_SIGNAL_DIRECTIONS> numPinsOfType;
	
	// Helper used for netlist traversals.
	int sign;
	
	// Mateus @ 20190204: Adding net use;
	Use netUse;
	
	NetData() : 
		mid(-1),
		sign(-1),
		driver(nullptr), 
		numPinsOfType({0, 0, 0, 0}),
		parent(nullptr),
		netUse(UNKNOWN_USE) {
	} // end constructor	
}; // end struct

} // end namespace