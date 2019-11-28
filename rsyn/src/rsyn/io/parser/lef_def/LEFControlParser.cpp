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

#include "LEFControlParser.h"

#ifndef WIN32
#include <unistd.h>
#else
#include <windows.h>
#endif /* not WIN32 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <map>
#include <iostream>

#include "rsyn/util/DoubleRectangle.h"
#include "rsyn/util/double2.h"
//LEF headers

#include "lef5.8/lefrReader.hpp"
#include "lef5.8/lefwWriter.hpp"
#include "lef5.8/lefiDebug.hpp"
#include "lef5.8/lefiUtil.hpp"

#define BOOST_POLYGON_NO_DEPS
#include <boost/polygon/polygon.hpp>

// -----------------------------------------------------------------------------

// =============================================================================
// LEF/DEF Common Function for parser
// =============================================================================

//void freeCB(void* name);
//void* mallocCB(size_t size);
//void* reallocCB(void* name, int size);

void* mallocCB(size_t size) { // DEF Parser version 5.8 
	return malloc(size);
}

void* mallocCB(int size) { // LEF Parser version 5.8 
	return malloc(size);
}

// -----------------------------------------------------------------------------

void* reallocCB(void* name, size_t size) { // DEF Parser version 5.8 
	return realloc(name, size);
}

void* reallocCB(void* name, int size) { // LEF Parser version 5.8 
	return realloc(name, size);
}

// -----------------------------------------------------------------------------

void freeCB(void* name) {
	free(name);
	return;
}

// -----------------------------------------------------------------------------



//LEF CALLBACKS
void lefCheckType(lefrCallbackType_e c);
int lefMacroBeginCB(lefrCallbackType_e c, const char* macroName, lefiUserData ud);
int lefMacroEndCB(lefrCallbackType_e c, const char* macroName, lefiUserData ud);
int lefMacroCB(lefrCallbackType_e c, lefiMacro* macro, lefiUserData ud);
int lefPinCB(lefrCallbackType_e c, lefiPin* pin, lefiUserData ud);
int lefSiteCB(lefrCallbackType_e c, lefiSite* site, lefiUserData ud);
int lefUnits(lefrCallbackType_e c, lefiUnits* units, lefiUserData ud);
int lefObstructionCB(lefrCallbackType_e c, lefiObstruction* obs, lefiUserData ud);
int lefLayerCB(lefrCallbackType_e c, lefiLayer* layer, lefiUserData ud);
int lefSpacingCB(lefrCallbackType_e c, lefiSpacing* spacing, lefiUserData ud);
int lefViaCb(lefrCallbackType_e typ, lefiVia* via, lefiUserData data);
int lefViaRuleCb(lefrCallbackType_e typ, lefiViaRule* via, lefiUserData data);

LefDscp &getLibraryFromUserData(lefiUserData userData) {
	return *((LefDscp *) userData);
} // end function

// =============================================================================
// LEF Function Implementation
// =============================================================================

void LEFControlParser::parseLEF(const std::string &filename, LefDscp & dscp) {
	//	int retStr = 0;  unused variable

	FILE* lefFile;
	int res;

	lefrInit();

	lefrReset();

	(void) lefrSetOpenLogFileAppend();

	//(void) lefrSetShiftCase(); // will shift name to uppercase if caseinsensitive
	// is set to off or not set

	lefrSetMacroBeginCbk(lefMacroBeginCB);
	lefrSetMacroEndCbk(lefMacroEndCB);
	lefrSetMacroCbk(lefMacroCB);
	lefrSetPinCbk(lefPinCB);
	lefrSetSiteCbk(lefSiteCB);
	lefrSetUnitsCbk(lefUnits);
	lefrSetUserData((void*) 3);
	lefrSetMallocFunction(mallocCB);
	lefrSetReallocFunction(reallocCB);
	lefrSetFreeFunction(freeCB);
	lefrSetObstructionCbk(lefObstructionCB);
	lefrSetLayerCbk(lefLayerCB);
	lefrSetSpacingCbk(lefSpacingCB);
	lefrSetViaCbk(lefViaCb);
	lefrSetViaRuleCbk(lefViaRuleCb);
	lefrSetRegisterUnusedCallbacks();

	// Open the lef file for the reader to read
	if ((lefFile = fopen(filename.c_str(), "r")) == 0) {
		printf("Couldn’t open input file ’%s’\n", filename.c_str());
		exit(1);
	}

	// Invoke the parser
	res = lefrRead(lefFile, filename.c_str(), (void*) &dscp);
	if (res != 0) {
		printf("LEF parser returns an error. #: %d \n", res);

		return;
	}
	//(void) lefrPrintUnusedCallbacks(fout);
	(void) lefrReleaseNResetMemory();
	fclose(lefFile);
} // end method 

// -----------------------------------------------------------------------------

void lefCheckType(lefrCallbackType_e c) {
	if (c >= 0 && c <= lefrLibraryEndCbkType) {
		// OK
	} else {

		printf("ERROR: callback type is out of bounds!\n");
	}
} // end call back 

// -----------------------------------------------------------------------------

int lefMacroBeginCB(lefrCallbackType_e c, const char* macroName, lefiUserData ud) {
	LefDscp & dscp = getLibraryFromUserData(ud);
	dscp.clsLefMacroDscps.resize(dscp.clsLefMacroDscps.size() + 1);
	return 0;
} // end function

// -----------------------------------------------------------------------------

int lefMacroEndCB(lefrCallbackType_e c, const char* macroName, lefiUserData ud) {
	return 0;
} // end function

// -----------------------------------------------------------------------------

int lefMacroCB(lefrCallbackType_e c, lefiMacro* macro, lefiUserData ud) {
	LefDscp & dscp = getLibraryFromUserData(ud);

	LefMacroDscp & lefMacro = dscp.clsLefMacroDscps.back();

	lefMacro.clsMacro->setClass(macro->macroClass());
	lefMacro.clsMacro->setName(macro->name());
	if (macro->hasOrigin()) {
		lefMacro.clsMacro->setOrigin(macro->originX(), macro->originY());
	}
	lefMacro.clsMacro->setSize(macro->sizeX(), macro->sizeY());
	if (macro->hasXSymmetry()) {
		lefMacro.clsMacro->setXSymmetry();
	} // end if 
	if (macro->hasYSymmetry()) {
		lefMacro.clsMacro->setYSymmetry();
	} // end if 
	if (macro->has90Symmetry()) {
		lefMacro.clsMacro->set90Symmetry();
	} // end if 
	return 0;
} // end function

// -----------------------------------------------------------------------------
int numWarningsInoutPins = 0;
int numWarningsTristatePins = 0;

int lefPinCB(lefrCallbackType_e c, lefiPin* pin, lefiUserData ud) {
	typedef boost::polygon::polygon_90_with_holes_data<double> Polygon90;
	typedef boost::polygon::polygon_traits<Polygon90>::point_type BoostPoint;
	// Skip power and ground pins...
	//if (strcmp(pin->use(), "GROUND") == 0) return 0;
	//if (strcmp(pin->use(), "POWER") == 0) return 0;

	LefDscp & dscp = getLibraryFromUserData(ud);
	LefMacroDscp & lefMacro = dscp.clsLefMacroDscps.back();
	lefMacro.clsPins.resize(lefMacro.clsPins.size() + 1);
	LefPinDscp & lefPin = lefMacro.clsPins.back();
	
	lefPin.clsPinName = pin->name();
	lefPin.clsPinDirection = pin->direction();
	lefPin.clsPinUse = pin->use();

	// WORKORUND to support inout data pin
	if (lefPin.clsPinDirection.compare("INOUT") == 0) {
		lefPin.clsPinDirection = "OUTPUT";
		//  if (numWarningsInoutPins < 10)
		//  	std::cout << "WARNING: Unsupported INOUT direction in data pin. "
		//  	<< lefPin.clsPinName << ". Pin direction is replaced to " << lefPin.clsPinDirection
		//  	<< " [LEF CONTROL PARSER]\n";
		numWarningsInoutPins++;
	} // end if
	// END WORKORUND to support inout data pin

	// Mateus @ 190108 -- WORKORUND to support tristate pin
	if (lefPin.clsPinDirection.compare("OUTPUT TRISTATE") == 0) {
		lefPin.clsPinDirection = "OUTPUT";
		if (numWarningsTristatePins < 10)
			std::cout << "WARNING: Ignoring TRISTATE OUTPUT statement in pin "
			<< lefPin.clsPinName << ". Pin direction is replaced to " << lefPin.clsPinDirection
			<< " [LEF CONTROL PARSER]\n";
		numWarningsTristatePins++;
	} // end if
	// END WORKORUND to support tristate data pin

	lefPin.clsHasPort = pin->numPorts() > 0;

	if (lefPin.clsHasPort)
		lefPin.clsPorts.reserve(pin->numPorts());

	for (int j = 0; j < pin->numPorts(); j++) {
		const lefiGeometries* geometry = pin->port(j);
		lefPin.clsPorts.push_back(LefPortDscp());

		LefPortDscp & lefPort = lefPin.clsPorts.back();
		int numGeo = geometry->numItems();
		lefPort.clsLefPortGeoDscp.reserve(numGeo);
		LefPortGeometryDscp * geoDscp = nullptr;
		for (int i = 0; i < numGeo; i++) {

			lefiGeomEnum geoType = geometry->itemType(i);
			lefiGeomRect* rect;
			DoubleRectangle * bound;
			lefiGeomPolygon * poly;
			LefPolygonDscp * polyDscp;
			double2 * point;

			std::vector< boost::polygon::rectangle_data<double> > rects;
			Polygon90 polygon90;
			std::vector<BoostPoint> pts;
			switch (geoType) {
				case lefiGeomLayerE:
					lefPort.clsLefPortGeoDscp.push_back(LefPortGeometryDscp());
					geoDscp = &lefPort.clsLefPortGeoDscp.back();
					geoDscp->clsMetalName = geometry->lefiGeometries::getLayer(i);
					break;
				case lefiGeomRectE:
					rect = geometry->getRect(i);
					geoDscp->clsBounds.resize(geoDscp->clsBounds.size() + 1);
					bound = &geoDscp->clsBounds.back();
					bound->updatePoints(rect->xl, rect->yl, rect->xh, rect->yh);
					break;
				case lefiGeomPolygonE:
					// Mateus @ 2018/09/13:
					// Support for polygon-shapped pins
					poly = geometry->getPolygon(i);

					for (int k = 0; k < poly->numPoints; k++) {
						pts.push_back(boost::polygon::construct<BoostPoint>(poly->x[k], poly->y[k]));
					} // end for 

					boost::polygon::set_points(polygon90, pts.begin(), pts.end());
					boost::polygon::get_rectangles(rects, polygon90);
					for (int k = 0; k < rects.size(); k++) {
						double xl = rects[k].get(boost::polygon::HORIZONTAL).low();
						double xh = rects[k].get(boost::polygon::HORIZONTAL).high();
						double yl = rects[k].get(boost::polygon::VERTICAL).low();
						double yh = rects[k].get(boost::polygon::VERTICAL).high();
						;
						geoDscp->clsBounds.resize(geoDscp->clsBounds.size() + 1);
						bound = &geoDscp->clsBounds.back();
						bound->updatePoints(xl, yl, xh, yh);
					} // end for
					// end Mateus @ 2018/09/13	
					break;
				default:
					std::cout << "WARNING: function " << __func__ << " does not supports pin geometry type in the LEF Parser Control.\n";
					break;
			} // end switch 
		} // end for
	} // end for 
	return 0;
} // end function

// -----------------------------------------------------------------------------

int lefSiteCB(lefrCallbackType_e c, lefiSite* site, lefiUserData ud) {
	LefDscp & dscp = getLibraryFromUserData(ud);
	dscp.clsLefSiteDscps.resize(dscp.clsLefSiteDscps.size() + 1);
	LefSiteDscp &lefSite = dscp.clsLefSiteDscps.back();
	lefSite.clsName = site->name();

	lefSite.clsHasClass = site->hasClass();
	if (site->hasClass()) {
		lefSite.clsSiteClass = site->siteClass();
	} // end if 
	lefSite.clsSize[X] = site->sizeX();
	lefSite.clsSize[Y] = site->sizeY();
	lefSite.clsSymmetry.clear();
	if (site->hasXSymmetry()) {
		lefSite.clsSymmetry.append("X");
	} // end if 
	if (site->hasYSymmetry()) {
		lefSite.clsSymmetry.append(" Y");
	} // end if 
	if (site->has90Symmetry()) {
		lefSite.clsSymmetry.append(" R90");
	} // end if 
	if (lefSite.clsSymmetry.empty()) {
		lefSite.clsSymmetry = INVALID_LEF_NAME;
	} // end if 

	return 0;
} // end function

// -----------------------------------------------------------------------------

int lefUnits(lefrCallbackType_e c, lefiUnits* units, lefiUserData ud) {
	LefDscp & dscp = getLibraryFromUserData(ud);
	LefUnitsDscp & lefUnits = dscp.clsLefUnitsDscp;

	if (units->hasDatabase()) {
		lefUnits.clsDatabase = (int) units->databaseNumber();
		lefUnits.clsHasDatabase = true;
	}

	return 0;
} // end function

// -----------------------------------------------------------------------------

int lefObstructionCB(lefrCallbackType_e c, lefiObstruction* obs, lefiUserData ud) {
	//std::cout << "Reading lef obstacles\n";
	LefDscp & dscp = getLibraryFromUserData(ud);
	LefMacroDscp & lefMacro = dscp.clsLefMacroDscps.back();

	lefiGeometries *geometry;
	lefiGeomRect *rect;

	geometry = obs->lefiObstruction::geometries();
	const int numItems = geometry->lefiGeometries::numItems();
	for (int i = 0; i < numItems; i++) {
		if (geometry->lefiGeometries::itemType(i) == lefiGeomLayerE) {
			lefMacro.clsObs.resize(lefMacro.clsObs.size() + 1);
			LefObsDscp & lefObs = lefMacro.clsObs.back();
			lefObs.clsMetalLayer = geometry->getLayer(i);
		} else if (geometry->lefiGeometries::itemType(i) == lefiGeomRectE) {
			LefObsDscp & lefObs = lefMacro.clsObs.back();
			rect = geometry->lefiGeometries::getRect(i);
			DoubleRectangle libRect = DoubleRectangle(rect->xl, rect->yl, rect->xh, rect->yh);
			lefObs.clsBounds.push_back(libRect);
		} // end if-else 
	} // end for 
	
//	if (lefMacro.clsMacroName == "bufx2" || lefMacro.clsMacroName == "BUFX2") {
//		std::cout << "#Obstacles" << lefMacro.clsObs.size() << "\n";
//	}
	return 0;
} // end method 

// -----------------------------------------------------------------------------

int lefLayerCB(lefrCallbackType_e c, lefiLayer* layer, lefiUserData ud) {
	LefDscp & dscp = getLibraryFromUserData(ud);
	lefiLayer* lefLayer = new lefiLayer();
	dscp.clsLefLayer.push_back(lefLayer);
	lefLayer->setName(layer->name());
	if (layer->lefiLayer::hasType()) {
		lefLayer->setType(layer->type());
	}
	if (layer->lefiLayer::hasPitch()) {
		lefLayer->setPitch(layer->pitch());
	}
	if (layer->lefiLayer::hasXYPitch()) {
		lefLayer->setPitchXY(layer->pitchX(), layer->pitchY());
	}
	if (layer->lefiLayer::hasWidth()) {
		lefLayer->setWidth(layer->width());
	}
	if (layer->lefiLayer::hasMinwidth()) {
		lefLayer->setMinwidth(layer->minwidth());
	}
	if (layer->lefiLayer::hasDirection()) {
		lefLayer->setDirection(layer->direction());
	}
	if (layer->lefiLayer::hasSpacingNumber()) {
		int numSpacing = layer->lefiLayer::numSpacing();
		for (int i = 0; i < numSpacing; ++i) {
			lefLayer->setSpacingMin(layer->spacing(i));
			if (layer->hasSpacingEndOfLine(i)) {
				lefLayer->setSpacingEol(layer->spacingEolWidth(i), layer->spacingEolWithin(i));
			}
			if (layer->hasSpacingParellelEdge(i)) {
				lefLayer->setSpacingParSW(layer->spacingParSpace(i), layer->spacingParWithin(i));
			}
		} // end for 
	} // end if 
	for (int i = 0; i < layer->lefiLayer::numSpacingTable(); ++i) {
		lefLayer->addSpacingTable();
		if (layer->lefiLayer::spacingTable(i)->isParallel()) {
			lefiParallel* parallel = layer->lefiLayer::spacingTable(i)->parallel();
			int numLength = parallel->numLength();
			int numWidth = parallel->numWidth();
			if (numLength > 0) {
				for (int iLength = 0; iLength < numLength; ++iLength) {
				    lefLayer->addNumber(parallel->length(iLength));
				}
				lefLayer->addSpParallelLength();
			}
			if (numWidth > 0) {
				for (int iWidth = 0; iWidth < numWidth; ++iWidth) {
                    lefLayer->addSpParallelWidth(parallel->width(iWidth));
                    for (int iLength = 0; iLength < numLength; ++iLength) {
                        lefLayer->addNumber(parallel->widthSpacing(iWidth, iLength));
                    }
                    lefLayer->addSpParallelWidthSpacing();
				}
			}
		}
	}
	if (layer->lefiLayer::hasArea()) {
		lefLayer->setArea(layer->area());
	}
	for (unsigned i = 0; static_cast<int>(i) < layer->lefiLayer::numProps(); ++i) {
		if (layer->lefiLayer::propIsNumber(i)) {
			lefLayer->addNumProp(layer->lefiLayer::propName(i), layer->lefiLayer::propNumber(i), layer->lefiLayer::propValue(i), layer->lefiLayer::propType(i));
		} else if (layer->lefiLayer::propIsString(i)) {
			lefLayer->addProp(layer->lefiLayer::propName(i), layer->lefiLayer::propValue(i), layer->lefiLayer::propType(i));
		} else {
			printf("ERROR: LEF layer prop %u is neither number nor string!\n", i);
		}
	}
	return 0;
} // end method 

// -----------------------------------------------------------------------------

int lefSpacingCB(lefrCallbackType_e c, lefiSpacing* spacing, lefiUserData ud) {
	LefDscp & dscp = getLibraryFromUserData(ud);
	dscp.clsLefSpacingDscps.resize(dscp.clsLefSpacingDscps.size() + 1);
	LefSpacingDscp & lefSpacing = dscp.clsLefSpacingDscps.back();
	lefSpacing.clsLayer1 = spacing->lefiSpacing::name1();
	lefSpacing.clsLayer2 = spacing->lefiSpacing::name2();
	lefSpacing.clsDistance = spacing->lefiSpacing::distance();
	return 0;
} // end method 

// -----------------------------------------------------------------------------

int lefViaCb(lefrCallbackType_e typ, lefiVia* via, lefiUserData data) {
	LefDscp & dscp = getLibraryFromUserData(data);
	dscp.clsLefViaDscps.resize(dscp.clsLefViaDscps.size() + 1);
	LefViaDscp & viaDscp = dscp.clsLefViaDscps.back();
	viaDscp.clsIsDefault = via->hasDefault();
	viaDscp.clsName = via->name();
	viaDscp.clsHasViaRule = via->hasViaRule();
	if (viaDscp.clsHasViaRule) {
		viaDscp.clsViaRuleName = via->viaRuleName();
		viaDscp.clsXCutSize = via->xCutSize();
		viaDscp.clsYCutSize = via->yCutSize();
		viaDscp.clsXCutSpacing = via->xCutSpacing();
		viaDscp.clsYCutSpacing = via->yCutSpacing();
		viaDscp.clsXBottomEnclosure = via->xBotEnc();
		viaDscp.clsYBottomEnclosure = via->yBotEnc();
		viaDscp.clsXTopEnclosure = via->xTopEnc();
		viaDscp.clsYTopEnclosure = via->yTopEnc();
		viaDscp.clsBottomLayer = via->botMetalLayer();
		viaDscp.clsCutLayer = via->cutLayer();
		viaDscp.clsTopLayer = via->topMetalLayer();
		if (via->hasOrigin()) {
			viaDscp.clsHasOrigin = true;
			viaDscp.clsXOrigin = via->xOffset();
			viaDscp.clsYOrigin = via->yOffset();
		} // end if 
		if (via->hasOffset()) {
			viaDscp.clsHasOffset = true;
			viaDscp.clsXBottomOffset = via->xBotOffset();
			viaDscp.clsYBottomOffset = via->yBotOffset();
			viaDscp.clsXTopOffset = via->xTopOffset();
			viaDscp.clsYTopOffset = via->yTopOffset();
		} // end if 
		if (via->hasRowCol()) {
			viaDscp.clsHasRowCol = true;
			viaDscp.clsNumCutRows = via->numCutRows();
			viaDscp.clsNumCutCols = via->numCutCols();
		} // end if 
	} else {
		if(via->hasResistance()) {
			viaDscp.clsHasResistance = true;
			viaDscp.clsCutResistance = via->resistance();
		} // end if 
		for (int i = 0; i < via->numLayers(); ++i) {
			std::string layerName = via->layerName(i);
			std::deque<LefViaGeometryDscp> & geoDscps = viaDscp.clsGeometries[layerName];
			for (int j = 0; j < via->numRects(i); ++j) {
				geoDscps.push_back(LefViaGeometryDscp());
				LefViaGeometryDscp & geoDscp = geoDscps.back();
				DoubleRectangle & bounds = geoDscp.clsBounds;
				bounds[LOWER][X] = via->xl(i, j);
				bounds[LOWER][Y] = via->yl(i, j);
				bounds[UPPER][X] = via->xh(i, j);
				bounds[UPPER][Y] = via->yh(i, j);
			} // end for 
		} // end for
	} // end if-else
	return 0;
} // end method 

// -----------------------------------------------------------------------------

int lefViaRuleCb(lefrCallbackType_e typ, lefiViaRule* via, lefiUserData data) {
	LefDscp & dscp = getLibraryFromUserData(data);
	dscp.clsLefViaRuleDscps.push_back(LefViaRuleDscp());
	LefViaRuleDscp & viaDscp = dscp.clsLefViaRuleDscps.back();
	viaDscp.clsName = via->name();
	viaDscp.clsIsDefault = via->hasDefault();
	viaDscp.clsIsGenerate = via->hasGenerate();
	std::vector<LefViaRuleLayerDscp> & layers = viaDscp.clsLayers;
	layers.resize(via->numLayers());
	for (int i = 0; i < via->numLayers(); i++) {
		LefViaRuleLayerDscp & layer = layers[i];
		lefiViaRuleLayer * viaLayer = via->layer(i);
		layer.clsName = viaLayer->name();
		if (viaLayer->hasDirection()) {
			layer.clsHasDirection = true;
			layer.clsIsHorizontal = viaLayer->isHorizontal();
			layer.clsIsVertical = viaLayer->isVertical();
		} // end if 
		if (viaLayer->hasEnclosure()) {
			layer.clsHasEnclosure = true;
			layer.clsEnclosure1 = viaLayer->enclosureOverhang1();
			layer.clsEnclosure2 = viaLayer->enclosureOverhang2();
		} // end if
		if (viaLayer->hasRect()) {
			layer.clsHasRect = true;
			DoubleRectangle & bounds = layer.clsRect;
			bounds[LOWER][X] = viaLayer->xl();
			bounds[LOWER][Y] = viaLayer->yl();
			bounds[UPPER][X] = viaLayer->xh();
			bounds[UPPER][Y] = viaLayer->yh();
		} // end if 
		if (viaLayer->hasResistance()) {
			layer.clsHasResistance = true;
			layer.clsCutResistance = viaLayer->resistance();
		} // end if 
		if (viaLayer->hasWidth()) {
			layer.clsHasWidth = true;
			layer.clsMinWidth = viaLayer->widthMin();
			layer.clsMaxWidth = viaLayer->widthMax();
		} // end if 
		if (viaLayer->hasSpacing()) {
			layer.clsHasSpacing = true;
			layer.clsXSpacing = viaLayer->spacingStepX();
			layer.clsYSpacing = viaLayer->spacingStepY();
		} // end if 
	} // end for 
	std::vector<std::string> & vias = viaDscp.clsVias;
	vias.reserve(via->numVias());
	for (int i = 0; i < via->numVias(); ++i) {
		vias.push_back(via->viaName(i));
	} // end for 
	return 0;
} // end method 

// -----------------------------------------------------------------------------
