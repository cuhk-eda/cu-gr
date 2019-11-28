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

#ifndef PHYSICALDESIGN_PHYSICALDESIGN_H
#define PHYSICALDESIGN_PHYSICALDESIGN_H

namespace Rsyn {

class PhysicalDesign : public Proxy<PhysicalDesignData> {
	friend class PhysicalService;

	template<typename _PhysicalObject, typename _PhysicalObjectReference, typename _PhysicalObjectExtension> friend class PhysicalAttributeBase;
	template<typename _PhysicalObject, typename _PhysicalObjectExtension> friend class PhysicalAttributeImplementation;


protected:
	//! @brief Internal constructs a Rsyn::PhysicalDesign object with a pointer to the Rsyn::PhysicalDesignData.
	//! @details The data and methods in physical design are split into two different objects. 
	//! One, the object that is passed as parameter to the methods contains only the methods implementation. 
	//! while the data is inherent from a Proxy object that have a pointer to the object that stores data. 

	PhysicalDesign(PhysicalDesignData * dsg) : Proxy(dsg) {
	}
public:
	//! @brief Constructs a Rsyn::PhysicalDesign object with null pointer to the Rsyn::PhysicalDesignData.
	//! @details The data and methods in physical design are split into two different objects. 
	//! One, the object that is passed as parameter to the methods contains only the methods implementation. 
	//! while the data is inherent from a Proxy object that have a pointer to the object that stores data. 

	PhysicalDesign() : Proxy(nullptr) {
	}
	//! @brief Constructs a Rsyn::PhysicalDesign object with null pointer to the Rsyn::PhysicalDesignData.
	//! @details The data and methods in physical design are split into two different objects. 
	//! One, the object that is passed as parameter to the methods contains only the methods implementation. 
	//! while the data is inherent from a Proxy object that have a pointer to the object that stores data. 

	PhysicalDesign(std::nullptr_t) : Proxy(nullptr) {
	}

	//! @brief	loading the technology library specified as LEF to the PhysicalDesign
	//! @param	library is a reference to the descriptor of the technology library inspired in LEF architecture.
	void loadLibrary(const LefDscp & library);


	//! @brief	loading the design specified as DEF format to the PhysicalDesign. 
	//! @details	PhysicalDesign stores the dimensions of design elements using LEF Database Units resolution. 
	//! @param	design is a DefDscp reference to the descriptor of the design inspired in DEF architecture.
	void loadDesign(const DefDscp & design);

	//! @brief	Initializes the Rsyn::PhysicalDesignData, the attributes to the Rsyn::Design elements and control parameters.
	//! @param	Rsyn::Json &params may be: 1) "clsEnablePhysicalPins" true enables Rsyn::PhysicalPin, 
	//! 2) "clsEnableMergeRectangles" true enables merging rectangle bounds to be merged. It does not work to bounds defined as polygon, and 
	//! 3) "clsEnableNetPinBoundaries" true enables storing the pins (Rsyn::Pin) that defines the Bound box boundaries of the nets.
	//! 4) "clsContestMode" {NONE, ICCAD15} enables legacy support to the contest benchmark.
	void initPhysicalDesign(Rsyn::Design dsg, const Rsyn::Json &params = {});

	//! @brief	Setting the net clock. Otherwise, it is defined as nullptr.
	void setClockNet(Rsyn::Net net);

	//! @brief	Updating the Bound Box of all design nets. 
	//! @param	skipClockNet default is value false. Otherwise, the Bound Box of the clock network is skipped to update and is not added to total HPWL.
	void updateAllNetBounds(const bool skipClockNet = false);

	//! @brief	Updating the Bound box of method parameter net of the Design.
	//! @param	net A valid net of the Design.
	void updateNetBound(Rsyn::Net net);

	//! @brief	Returns the Data base resolution. 
	//! @param	type 
	//! @details	type is an enum defined as: Rsyn::LIBRARY_DBU to technology library data base resolution, 
	//! Rsyn::DESIGN_DBU to the design data base resolution, or 
	//! Rsyn::MULT_FACTOR_DBU is the integer multiplier of the design data base and library data base resolutions 
	//! (e.g. 2000 = 1000 * 2 -> DESIGN_DBU = LIBRARY_DBU * MULT_FACTOR_DBU).
	//! @return DBU data base resolution of the parameter type.
	DBU getDatabaseUnits(const DBUType type) const;
	
	//! @brief Converts micron value into library database units
	DBU convertMicronToLibraryDatabaseUnits(const double value) const;
	
	//! @brief Converts micron value into design database units
	DBU convertMicronToDesignDatabaseUnits(const double value) const;
	
	//! @brief Returning a DBUxy object that has total nets Bound Box HPWL for abscissa (X dimension in DBUxy) and ordinate (Y dimension in DBUxy).
	DBUxy getHPWL() const;

	//! @param dim	It is an enum Dimension X or Y. 
	//! @brief	Returning total nets Bound Box HPWL for the parameter dim.
	DBU getHPWL(const Dimension dim) const;
	//! @brief	It returns the number of elements of the parameter type
	//! @param	type may be: 
	//! 1) PHYSICAL_FIXED      -> Returning total area of the fixed cells inside of core bounds.
	//! 2) PHYSICAL_MOVABLE    -> Returning total area of the movable cells inside of core bounds.
	//! 3) PHYSICAL_BLOCK      -> Returning total area of the block cells inside of the core bounds.
	//! 4) PHYSICAL_PORT       -> Returning total area of the circuit ports.
	//! 5) PHYSICAL_PLACEABLE  -> Returning total area of the rows.
	//! @return an integer number
	int getNumElements(const PhysicalType type) const;
	//! @brief	It returns the total area from elements of the parameter type
	//! @param	type may be: 
	//! PHYSICAL_FIXED      -> Returning total area of the fixed cells inside of core bounds
	//! PHYSICAL_MOVABLE    -> Returning total area of the movable cells inside of core bounds
	//! PHYSICAL_BLOCK      -> Returning total area of the block cells inside of the core bounds
	//! PHYSICAL_PORT       -> Returning total area of the circuit ports
	//! PHYSICAL_PLACEABLE  -> Returning total area of the rows
	//! @return The total area in DBU 
	DBU getArea(const PhysicalType type) const;

	//! @brief Returns true if the Rsyn::PhysicalPin was enabled. Otherwise, it returns false.
	//! @details For some circuits is irrelevant to initialized Rsyn::PhysicalPin. It will only consume memory.
	bool isEnablePhysicalPins() const;
	//! @brief Returns true if the rectangles that defines the boundary were merged in a small set of them.
	//! @details For some circuits, e.g. 2015 ICCAD contest, the boundaries of macros are defined by a set of rectangles. 
	//! They may be merged into a small set. 
	bool isEnableMergeRectangles() const;
	//! @brief Returns true if the pins that define the Bound box boundaries of the nets were enabled to be stored. 
	//! Otherwise, it returns false.
	//! @details For some circuits or some improvement algorithms 
	//! is irrelevant to store the pins that defined the boundaries of the bound box of the net. 
	//! It will only consume memory and runtime.
	bool isEnableNetPinBoundaries() const;

	//! @brief Rsyn::PhysicalLibraryPin related to the Rsyn::LibraryPin libPin.
	Rsyn::PhysicalLibraryPin getPhysicalLibraryPin(Rsyn::LibraryPin libPin) const;
	//! @brief Rsyn::PhysicalLibraryPin related to the Rsyn::Pin pin.
	Rsyn::PhysicalLibraryPin getPhysicalLibraryPin(Rsyn::Pin pin) const;
	//! @brief Rsyn::PhysicalLibraryCell related to the Rsyn::LibraryCell libCell.
	Rsyn::PhysicalLibraryCell getPhysicalLibraryCell(Rsyn::LibraryCell libCell) const;
	//! @brief Rsyn::PhysicalLibraryCell related to the Rsyn::Cell cell.
	Rsyn::PhysicalLibraryCell getPhysicalLibraryCell(Rsyn::Cell cell) const;

	//! @brief Update physical data of the Rsyn::PhysicalCell related to cell.
	void updatePhysicalCell(Rsyn::Cell cell);
	//! @brief Remove physical data of the Rsyn::PhysicalCell related to cell.
	void removePhysicalCell(Rsyn::Cell cell);
	//! @brief Rsyn::PhysicalCell related to the Rsyn::Cell cell
	Rsyn::PhysicalCell getPhysicalCell(Rsyn::Cell cell) const;
	//! @brief Rsyn::PhysicalCell related to the Rsyn::Pin pin
	Rsyn::PhysicalCell getPhysicalCell(Rsyn::Pin pin) const;
	//! @brief Rsyn::PhysicalInstance related to the Rsyn::Instance instance
	Rsyn::PhysicalInstance getPhysicalInstance(Rsyn::Instance instance) const;
	//! @brief Rsyn::PhysicalInstance related to the Rsyn::Pin pin
	Rsyn::PhysicalInstance getPhysicalInstance(Rsyn::Pin pin) const;
	//! @brief Rsyn::PhysicalPort related to the Rsyn::Port port
	Rsyn::PhysicalPort getPhysicalPort(Rsyn::Port port) const;
	//! @brief Rsyn::PhysicalPort related to the Rsyn::Pin pin
	Rsyn::PhysicalPort getPhysicalPort(Rsyn::Pin pin) const;
	//! @brief Rsyn::PhysicalModule related to the Rsyn::Module module
	Rsyn::PhysicalModule getPhysicalModule(Rsyn::Module module) const;
	//! @brief Rsyn::PhysicalModule related to the Rsyn::Pin pin
	Rsyn::PhysicalModule getPhysicalModule(Rsyn::Pin pin) const;
	//! @brief return total number of the movable physical cells.
	//! @return An integer that is the number of the movable physical cells.
	int getNumMovedCells() const;

	//! @brief Returns the object that store the physical data of the Rsyn::PhysicalDie.
	Rsyn::PhysicalDie getPhysicalDie() const;
	//! @brief if Rsyn::PhysicalPin was initialized, then returns a valid object. Otherwise, returns nullptr.
	//! @return Rsyn::PhysicalPin related to the Rsyn::Design Rsyn::Pin pin
	Rsyn::PhysicalPin getPhysicalPin(Rsyn::Pin pin) const;

	//! @brief Returns the pin displacement from its cell origen. 
	//! @details Pin displacement is the distance from cell origen to the pin position inside of cell.
	//! The cell origin is the LOWER and LEFT corner. 
	//! @return Pin displacement in a DBUxy. The distance unit is DBU.
	DBUxy getPinDisplacement(Rsyn::Pin pin) const;

	//! @brief Returns the pin position. The position is the summation of pin displacement and its cell position.
	DBUxy getPinPosition(Rsyn::Pin pin) const;
	//! @brief Returns the relaxed pin position. 
	//! @details If pin is related to a physical cell, 
	//! the pin position is the cell position. Otherwise, the pin position is the summation 
	//! of the pin displacement and its physical cell position.
	DBUxy getRelaxedPinPosition(Rsyn::Pin pin) const;
	//! @brief Returns the pin displacement for abscissa or ordinate.
	//! @details Pin displacement is the distance from cell origen to the pin position inside of cell.
	//! The cell origin is the LOWER and LEFT corner. 
	DBU getPinDisplacement(Rsyn::Pin pin, const Dimension dim) const;
	//! @brief Returns the pin position. The position is the summation of pin displacement and its cell position.
	DBU getPinPosition(Rsyn::Pin pin, const Dimension dim) const;

	//! @brief	Returns the Rsyn::PhysicalNet object related to the net parameter.
	Rsyn::PhysicalNet getPhysicalNet(Rsyn::Net net) const;

	//! @brief	Returns the Rsyn::PhysicalLayer object associated to the parameter layer name.
	Rsyn::PhysicalLayer getPhysicalLayerByName(const std::string & layerName);

	//! @brief	Returns the Rsyn::PhysicalLayer object associated to the parameter layer index.
	//! @details	The index is a integer from 0 to less than the number of layers.
	//!		If the the index number is outside of the ranger, than a null reference is returned. 
	Rsyn::PhysicalLayer getPhysicalLayerByIndex(const int index);

	//! @brief	Returns the Rsyn::PhysicalLayer object associated to the parameter layer index.
	//! @details	The index is a integer from 0 to less than the number of layers of the parameter type.
	//!		If the the index number is outside of the ranger, than a null reference is returned. 
	Rsyn::PhysicalLayer getPhysicalLayerByIndex(const Rsyn::PhysicalLayerType layerType, const int index);
	//! @brief	Returns the Rsyn::PhysicalSite object associated to the parameter site name.
	Rsyn::PhysicalSite getPhysicalSiteByName(const std::string &siteName);
	//! @brief	Returns the Rsyn::PhysicalRegion object associated to the parameter region name.
	Rsyn::PhysicalRegion getPhysicalRegionByName(const std::string &siteName);
	//! @brief	Returns the Rsyn::PhysicalGroup object associated to the parameter group name.
	Rsyn::PhysicalGroup getPhysicalGroupByName(const std::string &siteName);
	//! @brief	Returns the Rsyn::PhysicalVia object associated to the parameter vias name.
	Rsyn::PhysicalVia getPhysicalViaByName(const std::string &viaName);
	
	//! @brief	Returns the Rsyn::PhysicalViaRuleBase object associated to the parameter via's name.
	Rsyn::PhysicalViaRuleBase getPhysicalViaRuleBaseByName(const std::string &viaName);
	//! @brief	Returns the Rsyn::PhysicalViaRule object associated to the parameter via's name.
	Rsyn::PhysicalViaRule getPhysicalViaRuleByName(const std::string &viaName);
	//! @brief	Returns the Rsyn::PhysicalViaRuleGenerate object associated to the parameter via's name.
	Rsyn::PhysicalViaRuleGenerate getPhysicalViaRuleGenerateByName(const std::string &viaName);

	//! @brief	Returns the total number of layers. It is the summation of routing, overlap, cut, and so forth layers.
	int getNumLayers(const Rsyn::PhysicalLayerType type) const;
	int getNumLayers() const;
	//! @brief	Returns a reference to the vector of PhysicalLayers. 
	Range<ListCollection<PhysicalLayerData, PhysicalLayer>> allPhysicalLayers();

	//! @brief	Returns the total number of vias.
	std::size_t getNumPhysicalVias() const;
	//! @brief	Returns a vector reference to the vector of PhysicalVias. 
	const std::vector<Rsyn::PhysicalVia> & allPhysicalVias() const;

	
	
	std::size_t getNumPhysicalTracks()const;

	int getNumPhysicalTracks(Rsyn::PhysicalLayer layer) const;

	const std::vector<Rsyn::PhysicalTracks> & allPhysicalTracks() const;

	const std::vector<Rsyn::PhysicalTracks> & allPhysicalTracks(Rsyn::PhysicalLayer layer) const;

	bool hasPhysicalTracks(Rsyn::PhysicalLayer layer) const;

    const std::vector<Rsyn::PhysicalGCell>& allPhysicalGCell() const;

    const std::vector<Rsyn::PhysicalRoutingGrid> & allPhysicalRoutingGrids() const;

	Rsyn::PhysicalRoutingGrid getPhysicalRoutingGrid(Rsyn::PhysicalLayer layer) const;

	bool hasPhysicalRoutingGrid(Rsyn::PhysicalLayer layer) const;

	int getNumPhysicalRoutingGrids() const;
	
	//! @brief check if the orientations are equivalent based on symmetry.
	bool checkEquivalentOrientations(Rsyn::PhysicalSymmetry symmetry, Rsyn::PhysicalOrientation orient1, Rsyn::PhysicalOrientation orient2) const;

	//! @brief	Returns the total number of spacing objects.  
	std::size_t getNumPhysicalSpacing() const;
	//! @brief	Returns a reference to the vector of PhysicalSpacing. 
	Range<ListCollection<PhysicalSpacingData, PhysicalSpacing>> allPhysicalSpacing() const;

	//! @brief	Returns the total number of region objects.  
	std::size_t getNumPhysicalRegions() const;
	//! @brief	Returns a reference to the range list of PhysicalRegion. 
	std::vector<PhysicalRegion> & allPhysicalRegions() const;

	//! @brief	Returns the total number of group objects.  
	std::size_t getNumPhysicalGroups() const noexcept;
	//! @brief	Returns a reference to the range list of PhysicalGroup. 
	std::vector<PhysicalGroup> & allPhysicalGroups() const;
	//! @brief	Returns the total number of physical special nets.  
	std::size_t getNumPhysicalSpecialNets() const noexcept;
	//! @brief Returns a constant reference to a vector of physical special nets.
	std::vector<PhysicalSpecialNet> & allPhysicalSpecialNets() const;

	//! @brief Returns the row height. It is assumed all rows have the same height.
	//! The row height of the first row is returned.
	//I'm assuming all rows have the same height.
	DBU getRowHeight() const;
	//! @brief Returns the row site width. It is assumed all rows have the same site width.
	//! The row site width of the first row is returned.
	DBU getRowSiteWidth() const;
	//! @brief Returns the total number of row objects.
	std::size_t getNumRows() const;
	//! @brief Iterates over all Physical Rows. 
	Range<ListCollection<PhysicalRowData, PhysicalRow>> allPhysicalRows();

	//! @brief Returns the LayersViasManager
	//! @comment Given a layer is possible to get all vias which connect to the bottom or top routing layers.
	Rsyn::LayerViaManager getLayerViaManager() const;

protected:
	//! @brief initializes the Rsyn::PhysicalSite objects into Rsyn::PhysicalDesign.
	void addPhysicalSite(const LefSiteDscp & site);
	//! @brief initializes the Rsyn::PhysicalLayer objects into Rsyn::PhysicalDesign.
	Rsyn::PhysicalLayerData * addPhysicalLayer(lefiLayer* layer, Rsyn::PhysicalLayerData * lower);
	//! @brief initializes the Rsyn::PhysicalVia objects into Rsyn::PhysicalDesign.
	void addPhysicalVia(const LefViaDscp & via);
	//! @brief initializes the Rsyn::PhysicalViaRule objects
	void addPhysicalViaRule(const LefViaRuleDscp & via);
	//! @brief initializes the Rsyn::PhysicalLibraryCell objects into Rsyn::PhysicalDesign.
	Rsyn::LibraryCell addPhysicalLibraryCell(const LefMacroDscp& macro);
	//! @brief initializes the Rsyn::PhysicalLibraryPin objects of the library cell into 
	//! Rsyn::PhysicalDesign.
	void addPhysicalLibraryPin(Rsyn::LibraryCell libCell, const LefPinDscp& lefPin);
	//! @brief initializes the Rsyn::PhysicalInstance object as Rsyn::PhysicalCell into Rsyn::PhysicalDesign.
	void addPhysicalCell(Rsyn::Instance cell, const DefComponentDscp& component);
	//! @brief initializes the Rsyn::PhysicalInstance object as Rsyn::PhysicalPort into Rsyn::PhysicalDesign.
	void addPhysicalPort(Rsyn::Instance cell, const DefPortDscp& port);
	//! @brief initializes the Rsyn::PhysicalRow objects into Rsyn::PhysicalDesign.
	void addPhysicalRow(const DefRowDscp& defRow);
	//! @brief initializes the Rsyn::PhysicalRegion objects into Rsyn::PhysicalDesign.
	void addPhysicalRegion(const DefRegionDscp& defRegion);
	//! @brief initializes the Rsyn::PhysicalGroup objects into Rsyn::PhysicalDesign.
	void addPhysicalGroup(const DefGroupDscp& defGroup);
	//! @brief Initializes Rsyn::PhysicalNetObject into Ryn::PhysicalDesign.
	//! @warning Only initializes routed wires.
public: // Temporary for the contest debug
	void addPhysicalNet(const DefNetDscp & netDscp);
protected:
	//! @brief Initializes Rsyn::PhysicalSpecialNet into Ryn::PhysicalDesign.
	void addPhysicalSpecialNet(const DefNetDscp & specialNet);
	//! @brief Adds a track.
	void addPhysicalTracks(const DefTrackDscp &track);
	//! @brief Inits routing grid
	void initRoutingGrid();
	//! @brief Adds a gcell.
	void addPhysicalGCell(const DefGcellGridDscp &gcell);
	//! @brief Adds design via.
	void addPhysicalDesignVia(const DefViaDscp & via);
	//! @brief initializes the Rsyn::PhysicalSpacing objects into Rsyn::PhysicalDesign.
	void addPhysicalSpacing(const LefSpacingDscp & spacing);
	//! @brief initializes the Rsyn::PhysicalPin objects into Rsyn::PhysicalDesign.
	//! @todo Implements this method
	void addPhysicalPin();

	//! @brief Merges the rectangles that defines the boundary in a small set of them.
	//! @details For some circuits, e.g. 2015 ICCAD contest, the boundaries of macros are defined by a set of rectangles. 
	//! They may be merged into a small set.
	//! @warning works only for rectangles 
	void mergeBounds(const std::vector<Bounds> & source, std::vector<Bounds> & target, const Dimension dim = X);

	//! @brief inits the manager of the layers and vias.
	void initLayerViaManager();

private:
	//! @brief Returns the Rsyn::PhysicalRow unique identifier.
	PhysicalIndex getId(Rsyn::PhysicalRow phRow) const;

	//! @brief Returns the Rsyn::PhysicalLayer unique identifier.
	PhysicalIndex getId(Rsyn::PhysicalLayer phLayer) const;

	//! @brief Returns the Rsyn::PhysicalSpacing unique identifier.
	PhysicalIndex getId(Rsyn::PhysicalSpacing spacing) const;

public:
	//! @details Creates the physical object to handle the physical object extensions.
	//! The extension maps the physical object to a null reference.
	PhysicalAttributeInitializer createPhysicalAttribute();

	//! @details Creates the physical object to handle the physical object extensions.
	//! The extension maps the physical object to the default parameter data.
	template<typename DefaultPhysicalValueType>
	PhysicalAttributeInitializerWithDefaultValue<DefaultPhysicalValueType>
	createPhysicalAttribute(const DefaultPhysicalValueType &defaultValue);

public:

	//! @brief typedef for callback to the moved instances. 
	typedef std::function<void(Rsyn::PhysicalInstance instance) > PostInstanceMovedCallback;
	//! @brief list of registered call backs. 
	typedef std::list<std::tuple<int, PostInstanceMovedCallback>>::iterator PostInstanceMovedCallbackHandler;

	////////////////////////////////////////////////////////////////////////////
	// Placement
	////////////////////////////////////////////////////////////////////////////	
	//! @brief places the Rsyn::PhysicalCell at defined position.
	//! @warning Caution when using dontNotifyObservers.
	//! We can use it when you may expect the move to be rolled back, but it is
	//! not, recall to mark the cell as dirty.
	void placeCell(Rsyn::PhysicalCell physicalCell, const DBU x, const DBU y,
		Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool dontNotifyObservers = false);
	//! @brief places the Rsyn::PhysicalCell at defined position.
	//! @warning Caution when using dontNotifyObservers.
	//! We can use it when you may expect the move to be rolled back, but it is
	//! not, recall to mark the cell as dirty.
	void placeCell(Rsyn::Cell cell, const DBU x, const DBU y,
		Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool dontNotifyObservers = false);
	//! @brief places the Rsyn::PhysicalCell at defined position.
	//! @warning Caution when using dontNotifyObservers.
	//! We can use it when you may expect the move to be rolled back, but it is
	//! not, recall to mark the cell as dirty.
	void placeCell(Rsyn::PhysicalCell physicalCell, const DBUxy pos,
		Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool dontNotifyObservers = false);
	//! @brief places the Rsyn::PhysicalCell at defined position.
	//! @warning Caution when using dontNotifyObservers.
	//! We can use it when you may expect the move to be rolled back, but it is
	//! not, recall to mark the cell as dirty.
	void placeCell(Rsyn::Cell cell, const DBUxy pos,
		Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool dontNotifyObservers = false);
        
        //Place Port 
        
        Rsyn::PhysicalOrientation checkOrientation(Rsyn::PhysicalPort physicalPort, 
                const DBU x, const DBU y);
        
        DBUxy checkPosition(const DBU x, const DBU y);
        
        bool getPhysicalPortByName(std::string name, Rsyn::PhysicalPort &phPort);
        
        void placePort(Rsyn::PhysicalPort physicalPort, const DBU x, const DBU y, 
                Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
                const bool disableSnapping = false, const bool dontNotifyObservers = false);
        
        void placePort(Rsyn::Port port, const DBU x, const DBU y,
                Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool disableSnapping = false, const bool dontNotifyObservers = false);
        
        void placePort(Rsyn::PhysicalPort physicalPort, const DBUxy pos,
		Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool disableSnapping = false, const bool dontNotifyObservers = false);
        
        void placePort(Rsyn::Port port, const DBUxy pos,
		Rsyn::PhysicalOrientation orient = ORIENTATION_INVALID,
		const bool disableSnapping = false, const bool dontNotifyObservers = false);
	
	//! @brief set cell orientation.
	//! @warning Caution when using dontNotifyObservers.
	//! We can use it when you may expect the move to be rolled back, but it is
	//! not, recall to mark the cell as dirty.
	void setCellOrientation(Rsyn::PhysicalCell physicalCell, Rsyn::PhysicalOrientation orient,
		const bool dontNotifyObservers = false);
	//! @brief set cell orientation.
	//! @warning Caution when using dontNotifyObservers.
	//! We can use it when you may expect the move to be rolled back, but it is
	//! not, recall to mark the cell as dirty.
	void setCellOrientation(Rsyn::Cell cell, Rsyn::PhysicalOrientation orient,
		const bool dontNotifyObservers = false);
	
	void flipCell(Rsyn::PhysicalCell physicalCell, const bool dontNotifyObservers = false);
	
	void flipCell(Rsyn::Cell cell, const bool dontNotifyObservers = false);

	//! @brief Explicitly notify observer that a cell was moved. This is only necessary
	//! if "dontNotifyObservers = true" in "placeCell" methods.
	//! @todo Remove
	void notifyInstancePlaced(Rsyn::Instance instance, Rsyn::DesignObserver *ignoreObserver = nullptr);

	////////////////////////////////////////////////////////////////////////////
	// Routing
	////////////////////////////////////////////////////////////////////////////

	void setNetRouting(Rsyn::Net net, const PhysicalRouting &routing);
	void clearNetRouting(Rsyn::Net net);
	const PhysicalRouting &getNetRouting(Rsyn::Net net) const;
	bool isNetRouted(Rsyn::Net net) const;

	////////////////////////////////////////////////////////////////////////////
	// Notifications
	////////////////////////////////////////////////////////////////////////////	
public:

	//! @brief Registers an observer to be notified about changes in the
	//!        netlist.
	template<class T>
	void registerObserver(T *observer);

	//! @brief Unregisters an observer so it will no longer receives
	//!        notifications about changes in the netlist.
	void unregisterObserver(PhysicalDesignObserver *observer);

}; // end class 

} // end namespace 


#endif /* PHYSICALDESIGN_PHYSICALDESIGN_H */

