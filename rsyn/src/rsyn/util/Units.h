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
 
#ifndef RSYN_INTERNAL_UNITS_H
#define RSYN_INTERNAL_UNITS_H

#include <cassert>
#include <cmath>

namespace Rsyn {

enum Measure {
	MEASURE_INVALID = -1,
	
	MEASURE_POWER,       // mW
	MEASURE_CURRENT,     // mA
	MEASURE_CAPACITANCE, // fF
	MEASURE_RESISTANCE,  // kOhm
	MEASURE_TIME,        // ps     Note: Time(ps) = Resistance(kOhm) * Capacitance(fF)
	MEASURE_DISTANCE,    // dbu    Note: no prefix, not measured in meters
	MEASURE_VOLTAGE,     // V      Note: no prefix
	
	NUM_MEASURES
}; // end enum

enum UnitPrefix {
	YOCTO = -24,
	ZEPTO = -21,
	ATTO  = -18,
	FEMTO = -15,
	PICO  = -12,
	NANO  = -9,
	MICRO = -6,
	MILLI = -3,
	CENTI = -2,
	DECI  = -1,
	NO_UNIT_PREFIX = 0,
	DEKA  = 1,
	HECTO = 2,
	KILO  = 3,
	MEGA  = 6,
	GIGA  = 9,
	TERA  = 12,
	PETA  = 15,
	EXA   = 18,
	ZETTA = 21,
	YOTTA = 24
}; // end enum

class Units {
public:
	
	static UnitPrefix getInternalUnitPrefix(const Measure measure) {
		switch (measure) {
			case MEASURE_POWER: return MILLI;
			case MEASURE_CURRENT: return MILLI;
			case MEASURE_CAPACITANCE: return FEMTO;
			case MEASURE_RESISTANCE: return KILO;
			case MEASURE_TIME: return PICO;
			case MEASURE_DISTANCE: return NO_UNIT_PREFIX;
			case MEASURE_VOLTAGE: return NO_UNIT_PREFIX;
			default: assert(false);
		} // end switch
		return NO_UNIT_PREFIX; // just for warning sake
	} // end method
	
	static std::string getMeasureSymbol(const Measure measure) {
		switch (measure) {
			case MEASURE_POWER: return "W";
			case MEASURE_CAPACITANCE: return "C";
			case MEASURE_CURRENT: return "A";
			case MEASURE_RESISTANCE: return "Ohm";
			case MEASURE_TIME: return "s";
			case MEASURE_DISTANCE: return "dbu";
			case MEASURE_VOLTAGE: return "V";
			default: assert(false);
		} // end switch
		return ""; // just for warning sake		
	} // end method
	
	static std::string getUnitPrefixSymbol(const UnitPrefix prefix) {
		switch (prefix) {
			case YOCTO : return "y";
			case ZEPTO : return "z";
			case ATTO  : return "a";
			case FEMTO : return "f";
			case PICO  : return "p";
			case NANO  : return "n";
			case MICRO : return "u";
			case MILLI : return "m";
			case CENTI : return "c";
			case DECI  : return "d";
			case NO_UNIT_PREFIX : return "";
			case DEKA  : return "da";
			case HECTO : return "h";
			case KILO  : return "k";
			case MEGA  : return "M";
			case GIGA  : return "G";
			case TERA  : return "T";
			case PETA  : return "P";
			case EXA   : return "E";
			case ZETTA : return "Z";
			case YOTTA : return "Y";		
			default : assert(false);
		} // end switch
		return "";  // just for warning sake
	} // end method

	//! @brief Returns the default unit string given a measure. For instance,
	//!        return "fC" for capacitance and "mW" for power.
	static std::string getDefaultInternalUnitString(const Measure measure) {
		return getUnitPrefixSymbol(getInternalUnitPrefix(measure)) + getMeasureSymbol(measure);
	} // end method

	static double convertUnits(
			const double sourceValue,
			const UnitPrefix sourceUnitPrefix, 
			const UnitPrefix targetUnitPrefix) {
		return sourceValue * std::pow(10, sourceUnitPrefix - targetUnitPrefix);
	} // end method
	
	static double convertToInternalUnits(
			const Measure measure, 
			const double sourceValue,
			const UnitPrefix sourceUnitPrefix) {
		const UnitPrefix targetUnitPrefix = getInternalUnitPrefix(measure);
		return convertUnits(sourceValue, sourceUnitPrefix, targetUnitPrefix);
	} // end method

	static double convertFromInternalUnits(
			const Measure measure,
			const double sourceValue,
			const UnitPrefix targetUnitPrefix) {
		const UnitPrefix sourceUnitPrefix = getInternalUnitPrefix(measure);
		return convertUnits(sourceValue, sourceUnitPrefix, targetUnitPrefix);
	} // end method
	
}; // end class

} // end namespace

#endif

