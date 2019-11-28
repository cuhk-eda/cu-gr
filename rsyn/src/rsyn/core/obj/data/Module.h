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

struct ModuleData {
	Design design;
	
	// Pointers to instances and nets in this hierarchy level (objects above and
	// below this hierarchy level are not stored). We don't need to store
	// pin and arc pointers as they can be deduced from cell pointers. Nets
	// on the other may exist without connecting any cell (pin) so can not be 
	// completely deduced from cells.
	//
	// TODO: Find a better way to store module-specific data. It seems we are
	// spending too much memory to keep this.
	List<Instance> instances;
	List<Net> nets;
	
	// Ports.
	// Redundant with instances, but allow faster access to ports.
	List<Port> ports;
	std::set<Port> portsByDirection[Rsyn::NUM_SIGNAL_DIRECTIONS]; // TODO: unify these too
	
	// Used for some netlist traversing...
	mutable int sign;
	
	ModuleData() :
		sign(0) {}
	
}; // end struct

} // end namespace