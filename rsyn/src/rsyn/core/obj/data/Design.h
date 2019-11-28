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

struct DesignData {  
	std::string name;

	Rsyn::Module topModule;

	List<InstanceData> instances;
	List<PinData> pins;
	List<ArcData> arcs;
	List<NetData> nets;
        List<Cell> cells;
	
	List<LibraryCellData> libraryCells;
	List<LibraryPinData> libraryPins;
	List<LibraryArcData> libraryArcs;

	std::vector<std::string> instanceNames;
	std::vector<std::string> netNames;
	
	int anonymousInstanceId;
	int anonymousNetId;
	
	std::unordered_map<std::string, Instance> instanceMapping;
	std::unordered_map<std::string, Net> netMapping;
	std::unordered_map<std::string, LibraryCell> libraryCellMapping;
	
	std::array<LibraryCell, NUM_SIGNAL_DIRECTIONS> portLibraryCells;
	std::set<Cell> ports[NUM_SIGNAL_DIRECTIONS];
	
	std::array<int, NUM_INSTANCE_TYPES> instanceCount;
	
	std::vector<Pin> structuralStartpoints;
	std::vector<Pin> structuralEndpoints;	
	std::vector<Net> netsInTopologicalOrder;
	
	bool dirty;	
	bool initialized;
	
	// Used for some netlist traversing (e.g. update topological ordering)...
	int sign;	
	
	// Observers
	std::array<std::list<DesignObserver *>, NUM_DESIGN_EVENTS> observers;
	
	// Constructor
	DesignData() :
		initialized(false),
		dirty(false),
		anonymousInstanceId(0),
		anonymousNetId(0),
		instanceCount({0, 0, 0}),
		sign(0) {
	} // end constructor
}; // end class

} // end namespace