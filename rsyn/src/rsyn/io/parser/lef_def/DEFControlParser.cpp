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

#include "DEFControlParser.h"

#ifndef WIN32
#include <unistd.h>
#else
#include <windows.h>
#endif /* not WIN32 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// DEF headers
#include "def5.8/defiComponent.hpp"
#include "def5.8/defiNet.hpp"
#include "def5.8/defiRowTrack.hpp"

#include "def5.8/defwWriter.hpp"

#include "def5.8/defrReader.hpp"

extern void freeCB(void* name);
extern void* mallocCB(size_t size);
extern void* reallocCB(void* name, size_t size);

#include "rsyn/util/Bounds.h"
#include "rsyn/util/DoubleRectangle.h"

// -----------------------------------------------------------------------------

DEFControlParser::DEFControlParser() {
} // end constructor

// -----------------------------------------------------------------------------

DEFControlParser::~DEFControlParser() {

} // end destructor

// -----------------------------------------------------------------------------

//DEF CALLBACKS
char* defAddress(const char* in);
void defCheckType(defrCallbackType_e c);
int defCompf(defrCallbackType_e c, defiComponent* co, defiUserData ud);
int defComponentStart(defrCallbackType_e c, int num, defiUserData ud);
int defDesignName(defrCallbackType_e c, const char* string, defiUserData ud);
int defEndFunc(defrCallbackType_e c, void* dummy, defiUserData ud);
int defExt(defrCallbackType_e t, const char* c, defiUserData ud);
int defNetStart(defrCallbackType_e c, int num, defiUserData ud);
int defNetWires(defiNet* net, DefNetDscp& netDscp);
int defNet(defrCallbackType_e c, defiNet* net, defiUserData ud);
int defUnits(defrCallbackType_e c, double d, defiUserData ud);
int defVersion(defrCallbackType_e c, double d, defiUserData ud);
int defRow(defrCallbackType_e type, defiRow* rowInfo, defiUserData userData);
char* defOrientStr(int orient);
int defOrient(std::string orient);
int defDieArea(defrCallbackType_e typ, defiBox* box, defiUserData ud);
int defTrack(defrCallbackType_e typ, defiTrack * track, defiUserData data);
int defGCellGrid(defrCallbackType_e typ, defiGcellGrid * gCell, defiUserData data);
int defRegion(defrCallbackType_e type, defiRegion* region, defiUserData ud);
int defRegionStart(defrCallbackType_e c, int num, defiUserData ud);
int defGroupStart(defrCallbackType_e c, int num, defiUserData ud);
int defGroupMember(defrCallbackType_e type, const char* name, defiUserData userData);
int defGroupName(defrCallbackType_e type, const char* name, defiUserData ud);
int defGroups(defrCallbackType_e type, defiGroup *group, defiUserData ud);

int defPin(defrCallbackType_e, defiPin *pin, defiUserData ud);
int defSpecialNetStart(defrCallbackType_e c, int num, defiUserData ud);
int defSpecialNet(defrCallbackType_e c, defiNet* net, defiUserData ud);
int defViaStart(defrCallbackType_e, int number, defiUserData);
int defVia(defrCallbackType_e, defiVia *, defiUserData);

DefDscp &getDesignFromUserData(defiUserData userData) {
	return *((DefDscp *) userData);
} // end function

// =============================================================================
// DEF Function Implementation
// =============================================================================

void DEFControlParser::parseDEF(const std::string &filename, DefDscp &defDscp) {
	defrInit();
	defrReset();

	//defrSetAddPathToNet();
	defrSetComponentCbk(defCompf);
	defrSetDesignCbk(defDesignName);
	defrSetPinCbk(defPin);
	defrSetUnitsCbk(defUnits);
	defrSetVersionCbk(defVersion);
	defrSetRowCbk(defRow);
	defrSetComponentStartCbk(defComponentStart);
	defrSetDieAreaCbk(defDieArea);
	defrSetMallocFunction(mallocCB);
	defrSetReallocFunction(reallocCB);
	defrSetFreeFunction(freeCB);
	defrSetNetStartCbk(defNetStart);
	defrSetNetCbk(defNet);


	//defrSetSNetWireCbk();

	defrSetGroupsStartCbk(defGroupStart);
	defrSetGroupNameCbk(defGroupName);
	defrSetGroupMemberCbk(defGroupMember);
	defrSetGroupCbk(defGroups);
	defrSetTrackCbk(defTrack);
	defrSetGcellGridCbk(defGCellGrid);
	defrSetRegionStartCbk(defRegionStart);
	defrSetRegionCbk(defRegion);

	// register special net call backs
	defrSetSNetStartCbk(defSpecialNetStart);
	defrSetSNetCbk(defSpecialNet);

	// register via call backs
	defrSetViaStartCbk(defViaStart);
	defrSetViaCbk(defVia);

	//defrSetGcellGridWarnings(3);

	defrSetAddPathToNet();

	FILE * f;
	int res;

	(void) defrSetOpenLogFileAppend();



	if ((f = fopen(filename.c_str(), "r")) == 0) {
		printf("Couldn't open input file '%s'\n", filename.c_str());
		return;
	}
	// Set case sensitive to 0 to start with, in History & PropertyDefinition
	// reset it to 1.
	res = defrRead(f, filename.c_str(), (void*) &defDscp, 1);

	if (res)
		printf("Reader returns bad status. %s\n", filename.c_str());

	//(void) defrPrintUnusedCallbacks(fout);
	(void) defrReleaseNResetMemory();
	(void) defrUnsetNonDefaultCbk();
	(void) defrUnsetNonDefaultStartCbk();
	(void) defrUnsetNonDefaultEndCbk();
	defrClear();

}

// -----------------------------------------------------------------------------

char* defAddress(const char* in) {
	return ((char*) in);
} // end method

// -----------------------------------------------------------------------------

void defCheckType(defrCallbackType_e c) {
	if (c >= 0 && c <= defrDesignEndCbkType) {
		// OK
	} else {
		printf("ERROR: callback type is out of bounds!\n");
	}
} // end method

// -----------------------------------------------------------------------------

int defRow(defrCallbackType_e type, defiRow* rowInfo, defiUserData userData) {
	DefDscp &defDscp = getDesignFromUserData(userData);
	defDscp.clsRows.resize(defDscp.clsRows.size() + 1);
	DefRowDscp &defRow = defDscp.clsRows.back();
	defRow.clsName = rowInfo->name();
	defRow.clsSite = rowInfo->macro();
	defRow.clsOrientation = rowInfo->orientStr();
	defRow.clsOrigin[X] = static_cast<DBU> (rowInfo->x());
	defRow.clsOrigin[Y] = static_cast<DBU> (rowInfo->y());

	if (rowInfo->hasDoStep()) {
		defRow.clsNumX = static_cast<int> (rowInfo->xNum());
		defRow.clsNumY = static_cast<int> (rowInfo->yNum());
		defRow.clsStepX = static_cast<int> (rowInfo->xStep());
		defRow.clsStepY = static_cast<int> (rowInfo->yStep());
	} else if (rowInfo->hasDo()) {
		defRow.clsNumX = static_cast<int> (rowInfo->xNum());
		defRow.clsNumY = static_cast<int> (rowInfo->yNum());
		defRow.clsStepX = 0;
		defRow.clsStepY = 0;
	} else {
		defRow.clsNumX = 1;
		defRow.clsNumY = 1;
		defRow.clsStepX = 0;
		defRow.clsStepY = 0;
	} // end else
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defPin(defrCallbackType_e, defiPin *pin, defiUserData userData) {
	DefDscp &defDscp = getDesignFromUserData(userData);
	defDscp.clsPorts.resize(defDscp.clsPorts.size() + 1);
	DefPortDscp &defPin = defDscp.clsPorts.back();

	defPin.clsName = pin->pinName();
	defPin.clsNetName = pin->netName();
	defPin.clsDirection = pin->direction();
	defPin.clsPos[X] = pin->placementX();
	defPin.clsPos[Y] = pin->placementY();
	defPin.clsICCADPos = defPin.clsPos;
	defPin.clsOrientation = pin->orientStr();
	if (pin->hasLayer()) {
		defPin.clsLayerName = pin->layer(0);
		int xl, yl, xh, yh;
		pin->bounds(0, &xl, &yl, &xh, &yh);
		defPin.clsLayerBounds[LOWER][X] = xl;
		defPin.clsLayerBounds[LOWER][Y] = yl;
		defPin.clsLayerBounds[UPPER][X] = xh;
		defPin.clsLayerBounds[UPPER][Y] = yh;
		defPin.clsICCADPos += defPin.clsLayerBounds.computeCenter(); // legacy iccad15 contest pin position
	} // end if 
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defCompf(defrCallbackType_e c, defiComponent* co, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsComps.push_back(DefComponentDscp());
	DefComponentDscp &defComp = defDscp.clsComps.back();

	defComp.clsName = DEFControlParser::unescape(co->id());
	defComp.clsMacroName = co->name();
	defComp.clsIsFixed = co->isFixed();
	defComp.clsIsPlaced = co->isPlaced();
	defComp.clsPos[X] = co->placementX();
	defComp.clsPos[Y] = co->placementY();
	defComp.clsOrientation = co->placementOrientStr();

	return 0;
} // end method

// -----------------------------------------------------------------------------

int defComponentStart(defrCallbackType_e c, int num, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsComps.reserve(num);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defNetStart(defrCallbackType_e c, int num, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsNets.reserve(num);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defDesignName(defrCallbackType_e c, const char* string, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsDesignName = string;
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defNetWires(defiNet* net, DefNetDscp& netDscp) {
	netDscp.clsWires.resize(net->numWires());
	for (unsigned i = 0; static_cast<int>(i) < net->numWires(); ++i) {
		DefWireDscp & wireDscp = netDscp.clsWires[i];
		defiWire * wire = net->wire(i);
		wireDscp.clsWireType = wire->wireType();
		wireDscp.clsWireSegments.resize(wire->numPaths());
		for (unsigned j = 0; static_cast<int>(j) < wire->numPaths(); ++j) {
			DefWireSegmentDscp & segmentDscp = wireDscp.clsWireSegments[j];
			defiPath* path = wire->path(j);
			path->initTraverse();
			int pathId = path->next();
			int x, y, extension;
			DefRoutingPointDscp * point;
			while (pathId != DEFIPATH_DONE) {
				switch (pathId) {
					case DEFIPATH_LAYER:
						segmentDscp.clsLayerName = path->getLayer();
						break;
					case DEFIPATH_VIA:
						point->clsViaName = path->getVia();
						point->clsHasVia = true;
						break;
					case DEFIPATH_VIAROTATION:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_VIAROTATION at " << __func__ << "\n";
						} // end if
					} // end block

						break;
					case DEFIPATH_WIDTH:
						segmentDscp.clsRoutedWidth = static_cast<DBU> (path->getWidth());
						break;
					case DEFIPATH_POINT:
						path->getPoint(&x, &y);
						segmentDscp.clsRoutingPoints.emplace_back();
						point = &segmentDscp.clsRoutingPoints.back();
						point->clsPos = DBUxy(static_cast<DBU> (x), static_cast<DBU> (y));
						point->clsExtension = -1;
						break;
					case DEFIPATH_FLUSHPOINT:
						path->getFlushPoint(&x, &y, &extension);
						segmentDscp.clsRoutingPoints.emplace_back();
						point = &segmentDscp.clsRoutingPoints.back();
						point->clsPos = DBUxy(static_cast<DBU> (x), static_cast<DBU> (y));
						point->clsExtension = extension;
						point->clsHasExtension = true;
						break;
					case DEFIPATH_TAPER:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_TAPER at " << __func__ << "\n";
						} // end if
					} // end block

						break;
					case DEFIPATH_SHAPE:
						segmentDscp.clsHasShape = true;
						segmentDscp.clsShape = path->getShape();
						break;
					case DEFIPATH_MASK:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_MASK at " << __func__ << "\n";
						} // end if
					} // end block

						break;
					case DEFIPATH_VIAMASK:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_VIAMASK at " << __func__ << "\n";
						} // end if
					} // end block

						break;
					case DEFIPATH_RECT:
						int deltaxl, deltayl, deltaxu, deltayu;
						path->getViaRect(&deltaxl, &deltayl, &deltaxu, &deltayu);
						point->clsHasRectangle = true;
						point->clsRect[LOWER][X] = static_cast<DBU> (deltaxl);
						point->clsRect[LOWER][Y] = static_cast<DBU> (deltayl);
						point->clsRect[UPPER][X] = static_cast<DBU> (deltaxu);
						point->clsRect[UPPER][Y] = static_cast<DBU> (deltayu);

						break;
					case DEFIPATH_VIRTUALPOINT:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_VIRTUALPOINT at " << __func__ << "\n";
						} // end if
					} // end block

						break;
					case DEFIPATH_TAPERRULE:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_TAPERRULE at " << __func__ << "\n";
						} // end if
					} // end block

						break;
					case DEFIPATH_STYLE:

					{ // TEMPORARY WARNING MESSAGE // mateus @ 180528
						static bool warning = false;
						if (!warning) {
							warning = true;
							std::cout << "TODO DEFIPATH_STYLE at " << __func__ << "\n";
						} // end if
					} // end block

						break;
				} // end switch
				pathId = path->next();
			} // end while 
		} // end for 
	} // end for 
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defNet(defrCallbackType_e c, defiNet* net, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsNets.emplace_back();
	DefNetDscp & netDscp = defDscp.clsNets.back();
	netDscp.clsName = net->name();
	netDscp.clsConnections.resize(net->numConnections());
	for (unsigned i = 0; static_cast<int>(i) < net->numConnections(); ++i) {
		DefNetConnection &connection = netDscp.clsConnections[i];
		connection.clsPinName = net->pin(i);
		connection.clsComponentName = DEFControlParser::unescape(net->instance(i));
	} // end for
    defNetWires(net, netDscp);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defSpecialNetStart(defrCallbackType_e c, int num, void* ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsSpecialNets.reserve(num);
	return 0;
}// end method

// -----------------------------------------------------------------------------

int defSpecialNet(defrCallbackType_e c, defiNet* net, void* ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsSpecialNets.emplace_back();
	DefNetDscp & netDscp = defDscp.clsSpecialNets.back();
	netDscp.clsName = net->name();
	netDscp.clsConnections.resize(net->numConnections());
    if (net->hasUse()) {
		netDscp.clsHasUse = true;
		netDscp.clsUse = net->use();
	}
	for (unsigned i = 0; static_cast<int>(i) < net->numConnections(); ++i) {
		DefNetConnection &connection = netDscp.clsConnections[i];
		connection.clsPinName = net->pin(i);
		connection.clsComponentName = DEFControlParser::unescape(net->instance(i));
	} // end for
    defNetWires(net, netDscp);
	return 0;
}// end method

// -----------------------------------------------------------------------------

int defTrack(defrCallbackType_e typ, defiTrack * track, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsTracks.push_back(DefTrackDscp());
	DefTrackDscp & trackDscp = defDscp.clsTracks.back();
	trackDscp.clsDirection = track->macro();
	trackDscp.clsLocation = static_cast<DBU> (std::round(track->x()));
	trackDscp.clsNumTracks = static_cast<DBU> (std::round(track->xNum()));
	trackDscp.clsSpace = static_cast<DBU> (std::round(track->xStep()));
	trackDscp.clsLayers.reserve(track->numLayers());
	for (int i = 0; i < track->numLayers(); i++) {
		trackDscp.clsLayers.push_back(track->layer(i));
	} // end for 
	return 0;
} // end method 

// -----------------------------------------------------------------------------

int defViaStart(defrCallbackType_e c, int number, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsVias.reserve(number);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defVia(defrCallbackType_e c, defiVia * via, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsVias.push_back(DefViaDscp());
	DefViaDscp & viaDscp = defDscp.clsVias.back();
	viaDscp.clsName = via->name();
	if (via->hasViaRule()) {
		viaDscp.clsHasViaRule = true;
		char * name;
		int xSize, ySize, xCutSpacing, yCutSpacing, xBotEnc, yBotEnc, xTopEnc, yTopEnc;
		char * bottom;
		char * cut;
		char * top;
		via->viaRule(&name, &xSize, &ySize, &bottom, &cut, &top, &xCutSpacing, &yCutSpacing, &xBotEnc, &yBotEnc, &xTopEnc, &yTopEnc);
		viaDscp.clsViaRuleName = name;
		viaDscp.clsXCutSize = xSize;
		viaDscp.clsYCutSize = ySize;
		viaDscp.clsBottomLayer = bottom;
		viaDscp.clsCutLayer = cut;
		viaDscp.clsTopLayer = top;
		viaDscp.clsXCutSpacing = xCutSpacing;
		viaDscp.clsYCutSpacing = yCutSpacing;
		viaDscp.clsXBottomEnclosure = xBotEnc;
		viaDscp.clsYBottomEnclosure = yBotEnc;
		viaDscp.clsXTopEnclosure = xTopEnc;
		viaDscp.clsYTopEnclosure = yTopEnc;
		if (via->hasRowCol()) {
			viaDscp.clsHasRowCol = true;
			via->rowCol(&viaDscp.clsNumCutRows, &viaDscp.clsNumCutCols);
		} // end if 
		if (via->hasOrigin()) {
			viaDscp.clsHasOrigin = true;
			int xOrigen, yOrigen;
			via->origin(&xOrigen, &yOrigen);
			viaDscp.clsXOffsetOrigin = xOrigen;
			viaDscp.clsYOffsetOrigin = yOrigen;
		} // end if 
		if (via->hasOffset()) {
			viaDscp.clsHasOffset = true;
			int xBotOffset, yBotOffset, xTopOffset, yTopOffset;
			via->offset(&xBotOffset, &yBotOffset, &xTopOffset, &yTopOffset);
			viaDscp.clsXBottomOffset = xBotOffset;
			viaDscp.clsYBottomOffset = yBotOffset;
			viaDscp.clsXTopOffset = xTopOffset;
			viaDscp.clsYTopOffset = yTopOffset;
		} // end if 
		if (via->hasPattern()) {
			viaDscp.clsHasPattern = true;
			viaDscp.clsPattern = via->pattern();
		} // end if 
	} else {
		viaDscp.clsHasViaRule = false;
		std::map<std::string, std::deque < DefViaGeometryDscp>> &mapGeos = viaDscp.clsGeometries;
		int xl, yl, xh, yh;
		char * layerName;
		for (int i = 0; i < via->numLayers(); ++i) {
			via->layer(i, &layerName, &xl, &yl, &xh, &yh);
			std::string name = layerName;
			std::deque<DefViaGeometryDscp> & geos = mapGeos[name];
			geos.push_back(DefViaGeometryDscp());
			DefViaGeometryDscp & geoDscp = geos.back();
			geoDscp.clsIsRect = true;
			geoDscp.clsBounds[LOWER][X] = static_cast<DBU> (xl);
			geoDscp.clsBounds[LOWER][Y] = static_cast<DBU> (yl);
			geoDscp.clsBounds[UPPER][X] = static_cast<DBU> (xh);
			geoDscp.clsBounds[UPPER][Y] = static_cast<DBU> (yh);
			if (via->hasRectMask(i)) {
				geoDscp.clsHasMask = true;
				geoDscp.clsMask = via->rectMask(i);
			} // end if 
		} // end for 
	} // end if-else
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defUnits(defrCallbackType_e c, double d, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsDatabaseUnits = static_cast<int> (d);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defVersion(defrCallbackType_e c, double d, defiUserData ud) {
	DefDscp &defDscp = getDesignFromUserData(ud);
	defDscp.clsVersion = d;
	return 0;
} // end method

// -----------------------------------------------------------------------------

char* defOrientStr(int orient) {
	switch (orient) {
		case 0: return ((char*) "N");
		case 1: return ((char*) "W");
		case 2: return ((char*) "S");
		case 3: return ((char*) "E");
		case 4: return ((char*) "FN");
		case 5: return ((char*) "FW");
		case 6: return ((char*) "FS");
		case 7: return ((char*) "FE");
	};
	return ((char*) "BOGUS");
} // end method

// -----------------------------------------------------------------------------

int defOrient(std::string orient) {
	if (orient.compare("N") == 0) return 0;
	if (orient.compare("W") == 0) return 1;
	if (orient.compare("S") == 0) return 2;
	if (orient.compare("E") == 0) return 3;
	if (orient.compare("FN") == 0) return 4;
	if (orient.compare("FW") == 0) return 5;
	if (orient.compare("FS") == 0) return 6;
	if (orient.compare("FE") == 0) return 7;
	return -1;
} // end method

// -----------------------------------------------------------------------------

int defDieArea(defrCallbackType_e typ, defiBox* box, defiUserData ud) {
	DefDscp &defDscp = getDesignFromUserData(ud);
	defDscp.clsDieBounds[LOWER][X] = box->defiBox::xl();
	defDscp.clsDieBounds[LOWER][Y] = box->defiBox::yl();
	defDscp.clsDieBounds[UPPER][X] = box->defiBox::xh();
	defDscp.clsDieBounds[UPPER][Y] = box->defiBox::yh();
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defGCellGrid(defrCallbackType_e typ, defiGcellGrid * gcell, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsGcellGrids.emplace_back();
	DefGcellGridDscp & gcellGridDscp = defDscp.clsGcellGrids.back();
	gcellGridDscp.clsMacro = gcell->macro();
	gcellGridDscp.clsX = gcell->x();
	gcellGridDscp.clsXNum = gcell->xNum();
	gcellGridDscp.clsXStep = gcell->xStep();
	return 0;
} // end method 


// -----------------------------------------------------------------------------

int defRegionStart(defrCallbackType_e c, int num, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsRegions.reserve(num);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defRegion(defrCallbackType_e type, defiRegion* region, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsRegions.push_back(DefRegionDscp());
	DefRegionDscp & regionDscp = defDscp.clsRegions.back();
	regionDscp.clsName = region->name();
	regionDscp.clsType = region->type();
	regionDscp.clsBounds.resize(region->numRectangles());

	for (int i = 0; i < region->numRectangles(); i++) {
		Bounds & bounds = regionDscp.clsBounds[i];
		bounds[LOWER][X] = region->xl(i);
		bounds[LOWER][Y] = region->yl(i);
		bounds[UPPER][X] = region->xh(i);
		bounds[UPPER][Y] = region->yh(i);
	}
	return 0;
} // end method 

// -----------------------------------------------------------------------------

int defGroupStart(defrCallbackType_e c, int num, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsGroups.reserve(num);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defGroupName(defrCallbackType_e type, const char* name, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsGroups.push_back(DefGroupDscp());
	DefGroupDscp & defGroup = defDscp.clsGroups.back();
	defGroup.clsName = name;
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defGroupMember(defrCallbackType_e type, const char* name, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	DefGroupDscp & defGroup = defDscp.clsGroups.back();
	defGroup.clsPatterns.push_back(name);
	return 0;
} // end method

// -----------------------------------------------------------------------------

int defGroups(defrCallbackType_e type, defiGroup *group, defiUserData ud) {
	DefDscp & defDscp = getDesignFromUserData(ud);
	DefGroupDscp & defGroup = defDscp.clsGroups.back();
	defGroup.clsRegion = group->regionName();
	return 0;
} // end method 

// -----------------------------------------------------------------------------

#define CHECK_STATUS(status) \
	if (status) { \
		defwPrintError(status); \
	} // end if

// -----------------------------------------------------------------------------

void DEFControlParser::writeDEF(const std::string &filename, const std::string designName, const std::vector<DefComponentDscp> &components) {
	FILE * defFile;
	defFile = fopen(filename.c_str(), "w");
	if (defFile == NULL) {
		printf("ERROR: could not open output file: %s \n", filename.c_str());
	}

	int status;
	int numComponents = components.size();

	status = defwInitCbk(defFile);
	CHECK_STATUS(status);
	defwAddComment("ICCAD 2015 contest - CADA085 Team solution");

	status = defwNewLine();
	CHECK_STATUS(status);
	status = defwNewLine();
	CHECK_STATUS(status);

	status = defwVersion(5, 7);
	CHECK_STATUS(status);
	status = defwDesignName(designName.c_str());
	CHECK_STATUS(status);

	status = defwNewLine();
	CHECK_STATUS(status);

	status = defwStartComponents(numComponents);
	CHECK_STATUS(status);

	for (const DefComponentDscp & comp : components) {
		//int defwComponent(const char* name, const char* master, const char* eeq, const char* source, const char* status, int statusX, int statusY, int statusOrient, double weight, const char* region);
		status = defwComponent(comp.clsName.c_str(), comp.clsMacroName.c_str(),
			0, NULL, NULL, NULL, NULL, NULL, 0, NULL, NULL, NULL, NULL,
			comp.clsIsFixed ? "FIXED" : "PLACED", (int) comp.clsPos[X],
			(int) comp.clsPos[Y], 0, 0, NULL, 0, 0, 0, 0);
		CHECK_STATUS(status);
	}
	//defwEndComponents();
	status = defwEndComponents();
	CHECK_STATUS(status);

	status = defwNewLine();
	CHECK_STATUS(status);

	status = defwEnd();
	CHECK_STATUS(status);

	fclose(defFile);
} // end method 

// -----------------------------------------------------------------------------

void DEFControlParser::writeFullDEF(const string &filename, const DefDscp &defDscp) {
	//Opening 
	FILE * defFile;
	int status;
	defFile = fopen(filename.c_str(), "w");
	if (defFile == NULL) {
		printf("ERROR: could not open output file: %s \n", filename.c_str());
	}
	status = defwInitCbk(defFile);
	CHECK_STATUS(status);


	//writing header
	defwAddComment("DEF file generated by CUHK DR");

	status = defwNewLine();
	CHECK_STATUS(status);
	status = defwNewLine();
	CHECK_STATUS(status);
	status = defwVersion(5, 8); // TODO get from design
	CHECK_STATUS(status);
	status = defwDividerChar("/");
	CHECK_STATUS(status);
	status = defwBusBitChars("[]");
	CHECK_STATUS(status);
	status = defwDesignName(defDscp.clsDesignName.c_str());
	CHECK_STATUS(status);
	if (defDscp.clsHasDatabaseUnits) {
		status = defwUnits(defDscp.clsDatabaseUnits);
		CHECK_STATUS(status);
	} // end if 
	if (defDscp.clsHasDieBounds) {
		status = defwNewLine();
		CHECK_STATUS(status);
		status = defwDieArea(
			(int) defDscp.clsDieBounds[LOWER][X],
			(int) defDscp.clsDieBounds[LOWER][Y],
			(int) defDscp.clsDieBounds[UPPER][X],
			(int) defDscp.clsDieBounds[UPPER][Y]);
		CHECK_STATUS(status);
	} // end if 
	status = defwNewLine();
	CHECK_STATUS(status);

	for (const DefRowDscp & defRow : defDscp.clsRows) {
		status = defwRow(
			defRow.clsName.c_str(),
			defRow.clsSite.c_str(),
			defRow.clsOrigin[X],
			defRow.clsOrigin[Y],
			defOrient(defRow.clsOrientation),
			defRow.clsNumX,
			defRow.clsNumY,
			defRow.clsStepX,
			defRow.clsStepY);
		CHECK_STATUS(status);
		status = defwNewLine();
		CHECK_STATUS(status);
	} // end for 

	// Write tracks
	const char** layers;
	for (const DefTrackDscp & track : defDscp.clsTracks) {
		int numLayers = track.clsLayers.size();
		layers = (const char**) malloc(sizeof (char*)*numLayers);
		for (int i = 0; i < numLayers; i++) {
			layers[i] = track.clsLayers[i].c_str();
		} // end for 
		status = defwTracks(track.clsDirection.c_str(), track.clsLocation, track.clsNumTracks, track.clsSpace, numLayers, layers);
		CHECK_STATUS(status);
		free((char*) layers);
	} // end for 
	status = defwNewLine();
	CHECK_STATUS(status);

	// Write vias
	const unsigned numVias = defDscp.clsVias.size();
	if (numVias > 0) {
		status = defwStartVias(numVias);
		CHECK_STATUS(status);

		for (const DefViaDscp & via : defDscp.clsVias) {
			status = defwViaName(via.clsName.c_str());
			CHECK_STATUS(status);
			if (via.clsBottomLayer.size()) {
                std::map<std::string, std::deque<DefViaGeometryDscp>>::const_iterator it = via.clsGeometries.find(via.clsBottomLayer);
				if (it != via.clsGeometries.end()) {
					for (const DefViaGeometryDscp & geo : it->second) {
						const Bounds& bounds = geo.clsBounds;
						status = defwViaRect(via.clsBottomLayer.c_str(), bounds[LOWER][X], bounds[LOWER][Y], bounds[UPPER][X], bounds[UPPER][Y]);
						CHECK_STATUS(status);
					}
				}
			}
			if (via.clsTopLayer.size()) {
                std::map<std::string, std::deque<DefViaGeometryDscp>>::const_iterator it = via.clsGeometries.find(via.clsTopLayer);
				if (it != via.clsGeometries.end()) {
					for (const DefViaGeometryDscp & geo : it->second) {
						const Bounds& bounds = geo.clsBounds;
						status = defwViaRect(via.clsTopLayer.c_str(), bounds[LOWER][X], bounds[LOWER][Y], bounds[UPPER][X], bounds[UPPER][Y]);
						CHECK_STATUS(status);
					}
				}
			}
			if (via.clsCutLayer.size()) {
                std::map<std::string, std::deque<DefViaGeometryDscp>>::const_iterator it = via.clsGeometries.find(via.clsCutLayer);
				if (it != via.clsGeometries.end()) {
					for (const DefViaGeometryDscp & geo : it->second) {
						const Bounds& bounds = geo.clsBounds;
						status = defwViaRect(via.clsCutLayer.c_str(), bounds[LOWER][X], bounds[LOWER][Y], bounds[UPPER][X], bounds[UPPER][Y]);
						CHECK_STATUS(status);
					}
				}
			}
			if (via.clsHasViaRule) {
				status = defwNewLine();
				CHECK_STATUS(status);
				status = defwViaViarule(via.clsViaRuleName.c_str(), via.clsXCutSize, via.clsYCutSize, via.clsBottomLayer.c_str(), via.clsCutLayer.c_str(), via.clsTopLayer.c_str(), via.clsXCutSpacing, via.clsYCutSpacing, via.clsXBottomEnclosure, via.clsYBottomEnclosure, via.clsXTopEnclosure, via.clsYTopEnclosure);
				CHECK_STATUS(status);
			}
			if (via.clsHasRowCol) {
				status = defwViaViaruleRowCol(via.clsNumCutRows, via.clsNumCutCols);
				CHECK_STATUS(status);
			}
			if (via.clsHasOrigin) {
				status = defwViaViaruleOrigin(via.clsXOffsetOrigin, via.clsYOffsetOrigin);
				CHECK_STATUS(status);
			}
			if (via.clsHasOffset) {
				status = defwViaViaruleOffset(via.clsXBottomOffset, via.clsYBottomOffset, via.clsXTopOffset, via.clsYTopOffset);
				CHECK_STATUS(status);
			}
			if (via.clsHasPattern) {
				status = defwViaViarulePattern(via.clsPattern.c_str());
				CHECK_STATUS(status);
			}
			status = defwNewLine();
			CHECK_STATUS(status);
			status = defwOneViaEnd();
			CHECK_STATUS(status);
		} // end for 

		status = defwEndVias();
		CHECK_STATUS(status);
	} // end if 
	status = defwNewLine();
	CHECK_STATUS(status);

	// Write components
	const unsigned numComponents = defDscp.clsComps.size();
	if (numComponents > 0) {
		status = defwStartComponents(numComponents);
		CHECK_STATUS(status);

		for (const DefComponentDscp & comp : defDscp.clsComps) {
			int orient = defOrient(comp.clsOrientation);
			status = defwComponent(comp.clsName.c_str(), comp.clsMacroName.c_str(), 0, NULL, NULL, NULL, NULL, NULL, 0, NULL, NULL, NULL, NULL,
				comp.clsIsFixed ? "FIXED" : comp.clsIsPlaced ? "PLACED" : "UNPLACED",
				(int) comp.clsPos[X], (int) comp.clsPos[Y], orient, 0, NULL, 0, 0, 0, 0);
			CHECK_STATUS(status);
		} // end for 

		status = defwEndComponents();
		CHECK_STATUS(status);
	} // end if 

	// write ports
	status = defwStartPins(defDscp.clsPorts.size());
	CHECK_STATUS(status);

	for (const DefPortDscp & port : defDscp.clsPorts) {
		status = defwPin(port.clsName.c_str(),
			port.clsNetName.c_str(),
			(port.clsSpecial ? 1 : 0), // 0 ignores, 1 set net as special
			(port.clsDirection.compare(INVALID_DEF_NAME) == 0 ? NULL : port.clsDirection.c_str()),
			(port.clsUse.compare(INVALID_DEF_NAME) == 0 ? NULL : port.clsUse.c_str()),
			(port.clsLocationType.compare(INVALID_DEF_NAME) == 0 ? NULL : port.clsLocationType.c_str()),
			(int) port.clsPos[X],
			(int) port.clsPos[Y],
			defOrient(port.clsOrientation),
			(port.clsLayerName.compare(INVALID_DEF_NAME) == 0 ? NULL : port.clsLayerName.c_str()),
			(int) port.clsLayerBounds[LOWER][X],
			(int) port.clsLayerBounds[LOWER][Y],
			(int) port.clsLayerBounds[UPPER][X],
			(int) port.clsLayerBounds[UPPER][Y]);
		CHECK_STATUS(status);
	} // end for 

	status = defwEndPins();
	CHECK_STATUS(status);
	status = defwNewLine();
	CHECK_STATUS(status);
	status = defwNewLine();
	CHECK_STATUS(status);

	const unsigned numSpecialNets = defDscp.clsSpecialNets.size();
	if (numSpecialNets > 0) {
		status = defwStartSpecialNets(numSpecialNets);
		CHECK_STATUS(status);


		// write special nets
		for (const DefNetDscp & defNet : defDscp.clsSpecialNets) {
			status = defwSpecialNet(defNet.clsName.c_str());
			CHECK_STATUS(status);
			// Writing special net connections
			for (const DefNetConnection & defConn : defNet.clsConnections) {
				status = defwSpecialNetConnection(defConn.clsComponentName.c_str(), defConn.clsPinName.c_str(), 0);
				CHECK_STATUS(status);
			} // end for 
			if (!defNet.clsWires.empty()) {
				bool routed = true;
				status = defwSpecialNetPathStart("ROUTED");
				CHECK_STATUS(status);
				for (const DefWireDscp & wire : defNet.clsWires) {
					for (const DefWireSegmentDscp & segment : wire.clsWireSegments) {
						if (!routed) {
							status = defwSpecialNetPathStart("NEW");
							CHECK_STATUS(status);
						}
						routed = false;

						bool hasVia = false;
						std::string viaName = "";
						bool hasRect = false;
						Bounds rect;

						status = defwSpecialNetPathLayer(segment.clsLayerName.c_str());
						CHECK_STATUS(status);
						status = defwSpecialNetPathWidth(segment.clsRoutedWidth);
						CHECK_STATUS(status);
						if (segment.clsHasShape) {
							status = defwSpecialNetPathShape(segment.clsShape.c_str());
							CHECK_STATUS(status);
                        }
						for (const DefRoutingPointDscp & pt : segment.clsRoutingPoints) {
							double posX = pt.clsPos[X];
							double posY = pt.clsPos[Y];
							if (pt.clsHasExtension) {
								double ext = pt.clsExtension;
								status = defwSpecialNetPathPointWithWireExt(1, &posX, &posY, &ext);
								CHECK_STATUS(status);
							} else {
								status = defwSpecialNetPathPoint(1, &posX, &posY);
								CHECK_STATUS(status);
							} // end if-else
							if (pt.clsHasVia) {
								hasVia = true;
								viaName = pt.clsViaName;
							}
						} // end for 

						if (hasVia) {
							status = defwSpecialNetPathVia(viaName.c_str());
							CHECK_STATUS(status);
						}
					} // end for 

				} // end for 
				status = defwSpecialNetPathEnd();
				CHECK_STATUS(status);
			} // end if
			if (defNet.clsHasUse) {
				status = defwSpecialNetUse(defNet.clsUse.c_str());
				CHECK_STATUS(status);
			} // end if
			status = defwNewLine();
			CHECK_STATUS(status);
			status = defwSpecialNetEndOneNet();
			CHECK_STATUS(status);
		} // end for 

		status = defwEndSpecialNets();
		CHECK_STATUS(status);
	} // end if 

	const unsigned numNets = defDscp.clsNets.size();
	if (numNets > 0) {
		status = defwStartNets(numNets);
		CHECK_STATUS(status);
	

		// write nets
		for (const DefNetDscp & defNet : defDscp.clsNets) {
			status = defwNet(defNet.clsName.c_str());
			CHECK_STATUS(status);
			// Writing net connections
			for (const DefNetConnection & defConn : defNet.clsConnections) {
				status = defwNetConnection(defConn.clsComponentName.c_str(), defConn.clsPinName.c_str(), 0);
				CHECK_STATUS(status);
			} // end for 
			if (!defNet.clsWires.empty()) {
				bool routed = true;
				status = defwNetPathStart("ROUTED");
				CHECK_STATUS(status);
				for (const DefWireDscp & wire : defNet.clsWires) {
					for (const DefWireSegmentDscp & segment : wire.clsWireSegments) {
						if (!routed) {
							status = defwNetPathStart("NEW");
							CHECK_STATUS(status);
						}
						routed = false;

						bool hasVia = false;
						std::string viaName = "";
						bool hasRect = false;
						Bounds rect;

						status = defwNetPathLayer(segment.clsLayerName.c_str(), 0, NULL);
						CHECK_STATUS(status);
						for (const DefRoutingPointDscp & pt : segment.clsRoutingPoints) {
							double posX = pt.clsPos[X];
							double posY = pt.clsPos[Y];
							if (pt.clsHasExtension) {
								double ext = pt.clsExtension;
								status = defwNetPathPointWithExt(1, &posX, &posY, &ext);
								CHECK_STATUS(status);
							} else {
								status = defwNetPathPoint(1, &posX, &posY);
								CHECK_STATUS(status);
							} // end if-else
							if (pt.clsHasVia) {
								hasVia = true;
								viaName = pt.clsViaName;
							}
							if (pt.clsHasRectangle) {
								hasRect = true;
								rect = pt.clsRect;
							}
						} // end for 

						if (hasVia) {
							status = defwNetPathViaWithOrient(viaName.c_str(), -1);
							CHECK_STATUS(status);
						}
						if (hasRect) {
							status = defwNetPathRect(rect[LOWER][X], rect[LOWER][Y], rect[UPPER][X], rect[UPPER][Y]);
							CHECK_STATUS(status);
						}
					} // end for 

				} // end for
				status = defwNetPathEnd();
				CHECK_STATUS(status);
			} // end if 
			
			if (defNet.clsUse != INVALID_DEF_NAME) {
				status = defwNetUse(defNet.clsUse.c_str());
				status = defwNetPathEnd();
				CHECK_STATUS(status);
			} // end if 
			status = defwNetEndOneNet();
			CHECK_STATUS(status);
		} // end for 

		status = defwEndNets();
		CHECK_STATUS(status);
	} // end if 

	status = defwEnd();
	CHECK_STATUS(status);
	fclose(defFile);
} // end method

// -----------------------------------------------------------------------------

std::string DEFControlParser::unescape(const std::string &str) {
	std::string result;

	bool scapeNext = false;
	for (char ch : str) {
		if (scapeNext) {
			result += ch;
			scapeNext = false;
		} else if (ch == '\\') {
			scapeNext = true;
		} else {
			result += ch;
			scapeNext = false;
		} // end else
	} // end if

	return result;
} // end method

// -----------------------------------------------------------------------------

int defVia(defrCallbackType_e c, int num, void* ud) {
	std::cout << "Entered here!\n";
	DefDscp & defDscp = getDesignFromUserData(ud);
	defDscp.clsVias.reserve(num);
	return 0;
}// end method

// -----------------------------------------------------------------------------
