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

struct PinData : ObjectData {
	// #> bit pack
	int index : 20;           // index of this pin in the library cell.
	Direction direction : 3;  // direction of this pin
	InstanceType type : 3;    // the type of instance that this pin belongs to
	bool boundary : 1;        // Is this pin a port? Pins belonging to both
	                          // Rsyn::PORT and Rsyn::MODULE are marked as boundary
	                          // pins.
	// <#
	
	int sign;
	
	Instance instance;
	Net net;
	std::vector<Arc> arcs[NUM_TRAVERSE_TYPES];
	TopologicalIndex order; // topological ordering

	PinData() : 
		index(-1),
		direction(UNKNOWN_DIRECTION),
		type(UNKNOWN_INSTANCE_TYPE),
		boundary(false),
		sign(0),
		instance(nullptr), 
		net(nullptr), 
		
		order(std::numeric_limits<TopologicalIndex>::quiet_NaN()) {
	} // end constructor
}; // end struct

} // end namespace