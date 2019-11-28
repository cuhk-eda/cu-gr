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
 
#ifndef RSYN_LIBRARY_CELL_DESCRIPTOR_H
#define RSYN_LIBRARY_CELL_DESCRIPTOR_H

#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <tuple>
#include <unordered_map>

#include "lef5.8/lefiMacro.hpp"

namespace Rsyn {

class CellDescriptor {
friend class Design; // TODO: destroy this friendship...
private:
	// macro
	lefiMacro* macro;

	// Mateus @ 20180917: Adding PinUse
	// pin name, pin direction
	std::vector<std::tuple<std::string, Direction, Use>> pins;
	
	// from -> to
	std::vector<std::tuple<std::string, std::string>> arcs;
	
public:
	
	void setMacro(lefiMacro* macro) {
		this->macro = macro; // TODO: ugly
	} // end method
	
	// Mateus @ 20180917: Adding PinUse
	void addPin(const std::string &name, const Direction &direction, const Use& pinUse = UNKNOWN_USE) {
		pins.push_back(std::make_tuple(name, direction, pinUse));
	} // end method
	
	void addArc(const std::string &from, const std::string &to) {
		arcs.push_back(std::make_tuple(from, to));
	} // end method
	
	const std::string getName() const { 
		return macro->name(); 
	} // end method
	
	Rsyn::Direction getPinDirection(const std::string &name) const {
		const int numPins = pins.size();
		for (int i = 0; i < numPins; i++) {
			if (std::get<0>(pins[i]) == name) {
				return std::get<1>(pins[i]);
			} // end if
		} // end for
		return Rsyn::UNKNOWN_DIRECTION;
	} // end if
	
	void checkConsistency() const {
		const int numArcs = arcs.size();
		const int numPins = pins.size();
		
		std::map<std::string, Rsyn::Direction> mapPinToDirection;
		
		for (int i = 0; i < numPins; i++) {
			if ((std::get<1>(pins[i]) != Rsyn::IN) && (std::get<1>(pins[i]) != Rsyn::OUT)) {
				throw Exception("Only in and out direction are supported right now.");
			} // end if
			
			if (std::get<0>(pins[i]) == "") {
				throw Exception("Invalid pin name.");
			} // end if
			
			mapPinToDirection[std::get<0>(pins[i])] = std::get<1>(pins[i]);
		} // end for
		
		for (int i = 0; i < numArcs; i++) {
			const std::string &from = std::get<0>(arcs[i]);
			const std::string &to = std::get<1>(arcs[i]);
			
			if (from == to) {
				throw Exception("From and to pins are equal.");
			} // end if
			
			if (!mapPinToDirection.count(from)) {
				throw Exception("From pin not found.");
			} // end if

			if (!mapPinToDirection.count(to)) {
				throw Exception("To pin not found.");
			} // end if

			if (mapPinToDirection[from] != Rsyn::IN) {
				throw Exception("From pin is not an input pin.");
			} // end if
			
			if (mapPinToDirection[to] != Rsyn::OUT) {
				throw Exception("To pin is not an output pin.");
			} // end if
		} // end for		
		
	} // end method
	
}; // end class

} // end namespace

#endif
