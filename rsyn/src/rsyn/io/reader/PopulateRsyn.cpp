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
 
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PopulateRsyn.cpp
 * Author: jucemar
 * 
 * Created on 18 de Fevereiro de 2017, 15:21
 */

#include "PopulateRsyn.h"
namespace Rsyn {

// -----------------------------------------------------------------------------

void
PopulateRsyn::populateRsynLibraryFromLef(
	const LefDscp &lefDscp,
	Rsyn::Design rsynDesign
	) {
	for (const LefMacroDscp & macro : lefDscp.clsLefMacroDscps) {
		Rsyn::CellDescriptor dscp;
		dscp.setMacro(macro.clsMacro);
		for (const LefPinDscp &pin : macro.clsPins) {
			// Mateus @ 20180917: Adding PinUse
			dscp.addPin(pin.clsPinName, Legacy::lefPinDirectionFromString(pin.clsPinDirection), Legacy::lefPinUseFromString(pin.clsPinUse));
		} // end for
		rsynDesign.createLibraryCell(dscp, true);
	} // end for
} // end method

// -----------------------------------------------------------------------------

void PopulateRsyn::populateRsynNetlistFromVerilog(
	const Legacy::Design &verilogDesign,
	Rsyn::Design rsynDesign
	) {

	bool keepWarning;
	int keepWarningCounter;

	Rsyn::Module top = rsynDesign.getTopModule();
	rsynDesign.updateName(verilogDesign.name);

	keepWarning = true;
	keepWarningCounter = 0;
	for (auto &port : verilogDesign.primaryInputs) {
		Rsyn::Port rsynCell =
			rsynDesign.findPortByName(port);

		if (!rsynCell) {
			top.createPort(Rsyn::IN, port);
		} else {
			if (keepWarning) {
				if (++keepWarningCounter > 10) {
					keepWarning = false;
					std::cout << "[WARNING] At leat one more duplicated port...\n";
				} else {
					std::cout << "[WARNING] The primary input port '"
						<< port << "' is already defined.\n";
				} // end else
			} // end if
		} // end else
	} // end for

	keepWarning = true;
	keepWarningCounter = 0;

	for (auto &port : verilogDesign.primaryOutputs) {
		Rsyn::Port rsynCell =
			rsynDesign.findPortByName(port);

		if (!rsynCell) {
			top.createPort(Rsyn::OUT, port);
		} else {
			if (keepWarning) {
				if (++keepWarningCounter > 10) {
					keepWarning = false;
					std::cout << "[WARNING] At leat one more duplicated port...\n";
				} else {
					std::cout << "[WARNING] The primary output port '"
						<< port << "' is already defined.\n";
				} // end else
			} // end if
		} // end else
	} // end for

	// Creates cells.
	for (auto &component : verilogDesign.components) {
		Rsyn::LibraryCell lcell =
			rsynDesign.findLibraryCellByName(component.id);

		if (!lcell) {
            std::string str = "Library cell " + component.id + " not found\n";
			throw Exception(str);
		} // end if
		top.createCell(lcell, component.name);
	} // end for

	// Creates nets and connections.
	for (auto &net : verilogDesign.nets) {
		if (net.name == "") {
			std::cout << "[ERROR] Empty net name.\n";
			for (unsigned i = 0; i < net.connections.size(); i++) {
				std::cout << "Connection: " << net.connections[i].instance << ":" << net.connections[i].pin << "\n";
			} // end for
			exit(1);
		} // end if

		Rsyn::Net rsynNet =
			top.createNet(net.name);

		for (auto &connection : net.connections) {

			if (connection.instance == "PIN") {
				Rsyn::Port rsynCell =
					rsynDesign.findPortByName(connection.pin);

				if (!rsynCell) {
					std::cout << "[ERROR] The primary input/ouput port '"
						<< connection.pin << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin =
					rsynCell.getInnerPin();

				rsynPin.connect(rsynNet);

			} else {
				Rsyn::Cell rsynCell =
					rsynDesign.findCellByName(connection.instance);

				if (!rsynCell) {
					std::cout << "[ERROR] Cell '"
						<< connection.instance << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin =
					rsynCell.getPinByName(connection.pin);

				rsynPin.connect(rsynNet);

			} // end else
		} // end for
	} // end for
} // end method

// -----------------------------------------------------------------------------

void PopulateRsyn::populateRsyn(
	const LefDscp &lefDscp,
	const DefDscp &defDscp,
	const Legacy::Design &verilogDesign,
	Rsyn::Design rsynDesign) {

	bool keepWarning;
	int keepWarningCounter;

	Rsyn::Module top = rsynDesign.getTopModule();
	rsynDesign.updateName(defDscp.clsDesignName);

	// Create library cells.
	// Some cells (e.g. TIE) only appears in the LEF, not in the Liberty
	// at least for ICCAD 15 benchmarks. That's why we need to try to create
	// also cells using LEF.
	populateRsynLibraryFromLef(lefDscp, rsynDesign);

	// Creates ports.
	for (const DefPortDscp &port : defDscp.clsPorts) {

		const Rsyn::Direction direction =
			(port.clsDirection == "INPUT") ? Rsyn::IN : Rsyn::OUT;

		top.createPort(direction, port.clsName);
	} // end for

	keepWarning = true;
	keepWarningCounter = 0;
	for (auto &port : verilogDesign.primaryInputs) {
		Rsyn::Port rsynCell =
			rsynDesign.findPortByName(port);

		if (!rsynCell) {
			if (keepWarning) {
				if (++keepWarningCounter > 10) {
					keepWarning = false;
					std::cout << "[WARNING] At leat one more undefined port...\n";
				} else {
					std::cout << "[WARNING] The primary input port '"
						<< port << "' is defined in Verilog file but not in the DEF file.\n";
				} // end else
			} // end if

			top.createPort(Rsyn::IN, port);
		} // end if
	} // end for

	keepWarning = true;
	keepWarningCounter = 0;

	for (auto &port : verilogDesign.primaryOutputs) {
		Rsyn::Port rsynCell =
			rsynDesign.findPortByName(port);

		if (!rsynCell) {
			if (keepWarning) {
				if (++keepWarningCounter > 10) {
					keepWarning = false;
					std::cout << "[WARNING] At leat one more undefined port...\n";
				} else {
					std::cout << "[WARNING] The primary output port '"
						<< port << "' is defined in Verilog file but not in the DEF file.\n";
				} // end else
			} // end if

			top.createPort(Rsyn::OUT, port);
		} // end if
	} // end for

	// Creates cells.
	for (const DefComponentDscp &component : defDscp.clsComps) {
		Rsyn::LibraryCell lcell =
			rsynDesign.findLibraryCellByName(component.clsMacroName);

		if (!lcell) {
            std::string str = "Library cell " + component.clsMacroName + " not found\n";
			throw Exception(str);
		} // end if
		top.createCell(lcell, component.clsName);
	} // end for

	// Creates nets and connections.
	for (auto &net : verilogDesign.nets) {
		if (net.name == "") {
			std::cout << "[ERROR] Empty net name.\n";
			for (unsigned i = 0; i < net.connections.size(); i++) {
				std::cout << "Connection: " << net.connections[i].instance << ":" << net.connections[i].pin << "\n";
			} // end for
			exit(1);
		} // end if

		Rsyn::Net rsynNet =
			top.createNet(net.name);

		for (auto &connection : net.connections) {

			if (connection.instance == "PIN") {
				Rsyn::Port rsynCell =
					rsynDesign.findPortByName(connection.pin);

				if (!rsynCell) {
					std::cout << "[ERROR] The primary input/ouput port '"
						<< connection.pin << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin =
					rsynCell.getInnerPin();

				rsynPin.connect(rsynNet);

			} else {
				Rsyn::Cell rsynCell =
					rsynDesign.findCellByName(connection.instance);

				if (!rsynCell) {
					std::cout << "[ERROR] Cell '"
						<< connection.instance << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin =
					rsynCell.getPinByName(connection.pin);

				rsynPin.connect(rsynNet);

			} // end else
		} // end for
	} // end for
} // end method

// -----------------------------------------------------------------------------

void PopulateRsyn::populateRsyn(
	const LefDscp &lefDscp,
	const DefDscp &defDscp,
	Rsyn::Design rsynDesign) {

	bool keepWarning;
	int keepWarningCounter;

	Rsyn::Module top = rsynDesign.getTopModule();
	rsynDesign.updateName(defDscp.clsDesignName);

	// Create library cells.
	// Some cells (e.g. TIE) only appears in the LEF, not in the Liberty
	// at least for ICCAD 15 benchmarks. That's why we need to try to create
	// also cells using LEF.
	populateRsynLibraryFromLef(lefDscp, rsynDesign);

	// Creates ports.
	for (const DefPortDscp &port : defDscp.clsPorts) {

		const Rsyn::Direction direction =
			(port.clsDirection == "INPUT") ? Rsyn::IN : Rsyn::OUT;

		top.createPort(direction, port.clsName);
	} // end for


	//	// Creates cells.
	for (const DefComponentDscp &component : defDscp.clsComps) {
		Rsyn::LibraryCell lcell =
			rsynDesign.findLibraryCellByName(component.clsMacroName);

		if (!lcell) {
            std::string str = "Library cell " + component.clsMacroName + " not found\n";
			throw Exception(str);
		} // end if
		top.createCell(lcell, component.clsName);
	} // end for


	// Creates nets and connections.
	for (const DefNetDscp &net : defDscp.clsNets) {
		if (net.clsName == "") {
			std::cout << "[ERROR] Empty net name.\n";
			for (unsigned i = 0; i < net.clsConnections.size(); i++) {
				std::cout << "Connection: " << net.clsConnections[i].clsComponentName << ":" << net.clsConnections[i].clsPinName << "\n";
			} // end for
			exit(1);
		} // end if

		Rsyn::Net rsynNet = top.createNet(net.clsName);

		const std::string use = net.clsUse;
		if (use == "ANALOG") {
			rsynNet.setUse(Rsyn::ANALOG);
		} else if (use == "CLOCK") {
			rsynNet.setUse(Rsyn::CLOCK);
		} else if (use == "GROUND") {
			rsynNet.setUse(Rsyn::GROUND);
		} else if (use == "POWER") {
			rsynNet.setUse(Rsyn::POWER);
		} else if (use == "RESET") {
			rsynNet.setUse(Rsyn::RESET);
		} else if (use == "SCAN") {
			rsynNet.setUse(Rsyn::SCAN);
		} else if (use == "SIGNAL") {
			rsynNet.setUse(Rsyn::SIGNAL);
		} else if (use == "TIEOFF") {
			rsynNet.setUse(Rsyn::TIEOFF);
		} // end if
			
		for (const DefNetConnection &connection : net.clsConnections) {

			if (connection.clsComponentName == "PIN") {
				Rsyn::Port rsynCell =
					rsynDesign.findPortByName(connection.clsPinName);

				if (!rsynCell) {
					std::cout << "[ERROR] The primary input/ouput port '"
						<< connection.clsPinName << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin = rsynCell.getInnerPin();
				rsynPin.connect(rsynNet);
			} else {
				Rsyn::Cell rsynCell = rsynDesign.findCellByName(connection.clsComponentName);
				if (!rsynCell) {
					std::cout << "[ERROR] Cell '"
						<< connection.clsComponentName << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin = rsynCell.getPinByName(connection.clsPinName);
				rsynPin.connect(rsynNet);
			} // end else
		} // end for
	} // end for
	
	for (const DefNetDscp &net : defDscp.clsSpecialNets) {
		if (net.clsName == "") {
			std::cout << "[ERROR] Empty net name.\n";
			for (unsigned i = 0; i < net.clsConnections.size(); i++) {
				std::cout << "Connection: " << net.clsConnections[i].clsComponentName << ":" << net.clsConnections[i].clsPinName << "\n";
			} // end for
			exit(1);
		} // end if

		Rsyn::Net rsynNet = top.createNet(net.clsName);
		
		const std::string use = net.clsUse;
		if (use == "ANALOG") {
			rsynNet.setUse(Rsyn::ANALOG);
		} else if (use == "CLOCK") {
			rsynNet.setUse(Rsyn::CLOCK);
		} else if (use == "GROUND") {
			rsynNet.setUse(Rsyn::GROUND);
		} else if (use == "POWER") {
			rsynNet.setUse(Rsyn::POWER);
		} else if (use == "RESET") {
			rsynNet.setUse(Rsyn::RESET);
		} else if (use == "SCAN") {
			rsynNet.setUse(Rsyn::SCAN);
		} else if (use == "SIGNAL") {
			rsynNet.setUse(Rsyn::SIGNAL);
		} else if (use == "TIEOFF") {
			rsynNet.setUse(Rsyn::TIEOFF);
		} // end if
				
		for (const DefNetConnection &connection : net.clsConnections) {
			if (connection.clsComponentName == "PIN") {
				Rsyn::Port rsynCell =
					rsynDesign.findPortByName(connection.clsPinName);

				if (!rsynCell) {
					std::cout << "[ERROR] The primary input/ouput port '"
						<< connection.clsPinName << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin = rsynCell.getInnerPin();
				rsynPin.connect(rsynNet);
			} else if (connection.clsComponentName == "*") {
				for (Rsyn::Instance inst: rsynDesign.getTopModule().allInstances()) {
					Rsyn::Pin rsynPin = inst.getPinByName(connection.clsPinName);
					if (!rsynPin) {
						continue;
					} // end if
					rsynPin.connect(rsynNet);
				} // end for
			} else {
				Rsyn::Cell rsynCell = rsynDesign.findCellByName(connection.clsComponentName);
				if (!rsynCell) {
					std::cout << "[ERROR] Cell '"
						<< connection.clsComponentName << "' not found.\n";
					exit(1);
				} // end if

				Rsyn::Pin rsynPin = rsynCell.getPinByName(connection.clsPinName);
				rsynPin.connect(rsynNet);
			} // end else
		} // end for
	} // end for
} // end method

// -----------------------------------------------------------------------------

} // end namespace 

