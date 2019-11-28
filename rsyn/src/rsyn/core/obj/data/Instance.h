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

struct InstanceTagData {
	TristateFlag fixed;
	TristateFlag block;
}; // end struct

// -----------------------------------------------------------------------------

struct InstanceData : ObjectData {
	InstanceType type;
		
	std::vector<Pin> pins;
	std::vector<Arc> arcs;
	
	// Design (Cached)
	// @todo Remove.
	Design design;
	
	// The module where this instance is instantiated. If only null for the top
	// module.
	Module parent;
	
	// Local index inside it's parent module.
	Index mid;

	// Extra data for different types of instances.
	union {
		void * extra;
		
		ModuleData * moduleData;
		PinData * outerPin;
		LibraryCellData * lcell;
	}; // end union

	// User tag
	InstanceTagData tag;

	// Physical information.
	Bounds clsBounds;
	PhysicalOrientation clsOrientation;
	DBUxy clsPortPos; // only for port to define position. (@todo remove, use bounds instead).

	// Constructor
	InstanceData() : 
		design(nullptr),
		type(UNKNOWN_INSTANCE_TYPE),
		parent(nullptr),
		extra(nullptr),
		mid(-1),
		clsBounds(0, 0, 0, 0),
		clsOrientation(ORIENTATION_N), // @todo set to R0 by default
		clsPortPos(0, 0) {
	} // end constructor
}; // end struct

} // end namespace