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
 
#ifndef RSYN_PHYSICALDESIGN_PHYSICALDESIGNDATA_H
#define RSYN_PHYSICALDESIGN_PHYSICALDESIGNDATA_H


namespace Rsyn {

class PhysicalDesignData : public PhysicalObject {
	friend class PhysicalDesign;
public:
	Rsyn::Design clsDesign;
	Rsyn::Module clsModule;
	Rsyn::LayerViaManagerData clsLayerViaManager;

	std::list<std::function<void(Rsyn::Instance cell) >> callbackAddCreatePhysicalCell;
	std::list<std::function<void(Rsyn::Instance cell) >> callbackOnPreCellRemoveEvent;
	Rsyn::Attribute<Rsyn::LibraryPin, PhysicalLibraryPinData> clsPhysicalLibraryPins;
	Rsyn::Attribute<Rsyn::Pin, PhysicalPinData> clsPhysicalPins;
	Rsyn::Attribute<Rsyn::Instance, PhysicalInstanceData> clsPhysicalInstances;
	Rsyn::Attribute<Rsyn::LibraryCell, PhysicalLibraryCellData> clsPhysicalLibraryCells;
	Rsyn::Attribute<Rsyn::Net, PhysicalNetData> clsPhysicalNets;
	Rsyn::List<PhysicalRowData> clsPhysicalRows;
	Rsyn::List<PhysicalLayerData> clsPhysicalLayers;
	Rsyn::List<PhysicalSpacingData> clsPhysicalSpacing;
	std::vector<PhysicalRegion> clsPhysicalRegions;
	std::vector<PhysicalGroup> clsPhysicalGroups;
	std::vector<PhysicalSite> clsPhysicalSites;
	std::vector<PhysicalVia> clsPhysicalVias;
	std::vector<Rsyn::PhysicalViaRuleBase> clsPhysicalViaRuleBases;
	std::vector<Rsyn::PhysicalViaRule> clsPhysicalViaRules;
	std::vector<Rsyn::PhysicalViaRuleGenerate> clsPhysicalViaRuleGenerates;
	std::vector<PhysicalSpecialNet> clsPhysicalSpecialNets;
	std::vector<PhysicalTracks> clsPhysicalTracks;
	std::vector<PhysicalGCell> clsPhysicalGCell;
	std::vector<Rsyn::PhysicalRoutingGrid> clsPhysicalRoutingGrids;
	std::unordered_map<std::string, int> clsMapPhysicalSites;
	std::unordered_map<std::string, std::size_t> clsMapPhysicalRegions;
	std::unordered_map<std::string, std::size_t> clsMapPhysicalGroups;
	std::unordered_map<std::string, std::size_t> clsMapPhysicalSpecialNets;
	std::map<Rsyn::PhysicalLayer, std::vector<Rsyn::PhysicalTracks>> clsMapLayerToTracks;
	std::map<Rsyn::PhysicalLayer, Rsyn::PhysicalRoutingGrid> clsMapLayerToRoutingGrid;
	
	//From LEF file
	std::unordered_map<std::string, std::size_t> clsMapPhysicalLayers;
	std::unordered_map<std::string, std::size_t> clsMapPhysicalVias;
	std::unordered_map<std::string, std::size_t> clsMapPhysicalViaRuleBases;
	std::vector<std::size_t> clsPhysicalRoutingLayerIndeces;
	//std::vector<PhysicalSpacing> clsPhysicalSpacing;

	Rsyn::PhysicalDieData clsPhysicalDie; // total area of the circuit including core bound. 

	DBU clsTotalAreas[NUM_PHYSICAL_TYPES];
	int clsNumElements[NUM_PHYSICAL_TYPES];
	int clsNumLayers[NUM_PHY_LAYER];

	DBUxy clsHPWL;
	DBU clsDBUs[NUM_DBU]; // LEF and DEF data base units resolution and DEF/LEF multiplier factor

	bool clsLoadDesign : 1;
	bool clsEnablePhysicalPins : 1;
	bool clsEnableMergeRectangles : 1;
	bool clsEnableNetPinBoundaries : 1;

	Rsyn::Net clsClkNet;
	
	// Physical design mode 
	PhysicalDesignMode clsMode = PhysicalDesignMode::ALL;
		
	////////////////////////////////////////////////////////////////////////////
	// Observerss
	////////////////////////////////////////////////////////////////////////////

	std::array<std::list<PhysicalDesignObserver *>, NUM_PHYSICAL_EVENTS> clsPhysicalObservers;

	PhysicalDesignData() : clsClkNet(nullptr), clsDesign(nullptr), clsModule(nullptr) {

		clsLoadDesign = false;
		clsEnablePhysicalPins = false;
		clsEnableMergeRectangles = false;
		clsEnableNetPinBoundaries = false;
		for (int index = 0; index < NUM_DBU; index++) {
			clsDBUs[index] = 0;
		} // end for 
		for (int index = 0; index < NUM_PHYSICAL_TYPES; index++) {
			clsTotalAreas[index] = 0.0;
			clsNumElements[index] = 0;
		} // end for  
		for (int index = 0; index < NUM_PHY_LAYER; index++) {
			clsNumLayers[index] = 0;
		}
	} // end constructor 
}; // end class 

} // end namespace

#endif
