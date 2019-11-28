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
 
#ifndef PLACER_INTERNALS_H
#define	PLACER_INTERNALS_H

#include <string>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// Design Descriptor
////////////////////////////////////////////////////////////////////////////////
#include "rsyn/util/Bounds.h"
#include "rsyn/util/dbu.h"

namespace Legacy {

struct Design {
	struct Component {
		std::string id;
		std::string name;
		bool fixed;
		bool placed;
		double x;
		double y;
	};

	struct Connection {
		std::string pin;
		std::string instance;
	};
	
	struct Net {
		std::string name;
		std::vector<Connection> connections;
	};

	struct Row {
		double x;
		double y;
		int numSitesX;
		int numSitesY;
		std::string site;
		std::string name;
		double stepX;
		double stepY;
	};
	
	struct Pin {
		std::string name;
		std::string net;
		std::string metal;
		std::string direction;
		double x;
		double y;
		DBUxy routingLower;
		DBUxy routingUpper;
	};
	
	struct DieArea {
		int xmin;
		int ymin;
		int xmax;
		int ymax;
	};

	struct Track {
		std::string direction; //X or Y
		int doStart;
		int doCount;
		int doStep;
		int numLayers;
		std::vector<std::string> layers;
		Track () : direction("NULL"), doStart(-1), doCount(-1), doStep(-1),
		numLayers(-1) {}
	};
	
	struct GCellGrid {
		std::string master; // direction X or Y
		int doStart;
		int doCount;
		int doStep;
		GCellGrid () : master("null"), doStart(-1), doCount(-1), doStep(-1) {}
	};
	
	std::string name;
	double distanceUnit;
	double defVersion;
	
	std::vector<Component> components;
	std::vector<Pin> ports;
	std::vector<Net> nets;
	std::vector<Row> rows;
	
	std::vector<std::string> primaryInputs;
	std::vector<std::string> primaryOutputs;
	
	std::vector<Track> tracks;
	std::vector<GCellGrid> gCellGrids;
	
	DieArea dieArea;
	
	Design() {
		distanceUnit = 1;
		defVersion = -1;
	}
}; // end struct

////////////////////////////////////////////////////////////////////////////////
// Library Descriptor
////////////////////////////////////////////////////////////////////////////////

struct Library {
	struct Site {
		std::string name;
		std::string classSite;
		bool hasClass;
		double w;
		double h;
		
		Site() :name("null"), classSite("null"), hasClass(false), w(0), h(0) {}
	};
	
	struct Pin {
		std::string name;
		std::string direction;
		std::string metalLayer;
		Bounds bound;
		std::vector<Bounds> routerPins;
		double dx;
		double dy;
		
		Pin() : dx(0), dy(0), metalLayer("null") {}
	};
	
	struct Obstruction {
		std::string layer; 
		std::vector<Bounds> rects;
	};
	
	struct Macro {
		std::string name;
		std::string type;
		std::string site;
		double w;
		double h;
		std::vector<Pin> pins;
		std::vector<Obstruction> obs;
		Macro () : w(0), h(0) {}
	}; // end struct
	
	struct Layer { // for metals, vias and contacts
		std::string name;
		std::string type;
		std::string direction;
		double pitch;
		double width;
		double spacing;
		Layer () : name("null"), type("null"), direction("null"), 
		pitch(-1), width(-1), spacing(-1) {}
	};
	
	struct Spacing {
		std::string name1;
		std::string name2;
		double distance;
	};
		
	std::string name;
	double distanceUnit;
	std::vector<Macro> macros;
	std::vector<Site> sites;
	std::vector<Layer> layers;
	std::vector<Spacing> metalSpaces;
	
	Library() : distanceUnit(1) {}	
}; // end struct


struct DefStruct1 {
	
	struct DefMacro {
		std::string  name;
		std::string lefCellName;
		bool placed;
		bool fixed;
		DBUxy pos;
	};
	
	struct DefPin {
		std::string pinName;
		std::string cellName;
	};
	
	struct DefConnection {
		std::string netName;
		std::vector<DefPin> pins; //cell and pin
	};
	
	struct DefRow {
		std::string rowName;
		std::string rowType;
		int origX;
		int origY;
		int orient;
		int doCount;
		int doIncrement;
		int xStep;
		int yStep;
		DefRow () : rowType("core"), origX(0), origY(0), orient(0), doCount(0), doIncrement(0), xStep(0), yStep(0) {}
		// orient {0, 1, 2, 3, 4, 5, 6, 7} = {N, W, S, E, FN, FW, FS, FE} source: defapi.pdf version 5.8 
	};

	struct DefPortLayer {
		std::string layerName;
		int spacing;
		int designRuleWidth;
		Bounds bounds;
		DefPortLayer() : spacing(0), designRuleWidth(0), layerName("NULL") {}
	};

	struct DefPort {
		std::string portName;
		std::string netName;
		int special; // 0 is ignored; 1 writes a SPECIAL statement
		std::string direction; // {NULL, FEEDTHRU, INPUT, INOUT, OUTPUT}
		std::string use; // {NULL, ANALOG, CLOCK, GROUND, POWER, RESET, SCAN, SIGNAL, TIEOFF}
		std::string status; // {NULL, COVER, FIXED, PLACED} 
		int statusX; // placement location; 0 ignore
		int statusY; // placement location; 0 ignore
		int orient;  // orient {-1, 0, 1, 2, 3, 4, 5, 6, 7} = {IGNORE, N, W, S, E, FN, FW, FS, FE} source: defapi.pdf version 5.8 
		DefPortLayer clsDefPortLayer;
		DefPort () : direction("NULL"), orient(-1), special(0), status("NULL"), use("NULL") {}
	};

	std::vector<DefMacro> clsDefMacros;
	std::vector<DefPort> clsDefPorts;
	std::vector<DefConnection> clsDefConnections;
	std::vector<DefRow> clsDefRows;
	
	int majorDefVersion;
	int minorDefVersion;
	std::string devideChar;
	std::string busBitChar;
	std::string designName;
	int designUnits;
	Bounds dieArea;
	
}; // end struct

} // end namespace

#endif

