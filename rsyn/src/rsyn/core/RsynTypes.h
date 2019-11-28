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
 
#ifndef RSYN_TYPES_H
#define RSYN_TYPES_H

#include <cstdint>

namespace Rsyn {

enum ObjectType {
	OBJECT_TYPE_INVALID = -1,

	OBJECT_TYPE_NET,
	OBJECT_TYPE_PIN,
	OBJECT_TYPE_ARC,

	OBJECT_TYPE_INSTANCE,
	OBJECT_TYPE_CELL,
	OBJECT_TYPE_PORT,
	OBJECT_TYPE_MODULE,

	OBJECT_TYPE_LIBRARY_PIN,
	OBJECT_TYPE_LIBRARY_ARC,
	OBJECT_TYPE_LIBRARY_CELL,

	NUM_OBJECT_TYPES
}; // end enum

enum InstanceType {
	UNKNOWN_INSTANCE_TYPE = 0,  // used in bit fields (needs to start at zero)

	CELL,
	PORT,
	MODULE,

	NUM_INSTANCE_TYPES
}; // end enum

enum ArcType {
	UNKNOWN_ARC_TYPE = 0, // used in bit fields (needs to start at zero)

	INSTANCE_ARC,
	NET_ARC,

	NUM_ARC_TYPES
}; // end enum

enum TraverseType {
	INVALID_TRAVERSE_TYPE = -1,

	FORWARD,  // from inputs to outputs (topological order)
	BACKWARD, // from outputs to inputs (reverse topological order)

	NUM_TRAVERSE_TYPES
}; // end enum

enum Direction { // TODO: Many rename to SignalDirection
	UNKNOWN_DIRECTION  = 0, // used in bit fields (needs to start at zero)

	IN = 1,
	SINK = 1,

	OUT = 2,
	DRIVER = 2,

	BIDIRECTIONAL = 3,

	NUM_SIGNAL_DIRECTIONS
}; // end enum

//! @brief Orientation based on compass rose.
//! @details The default orientation is "vertically and face up" - N (North).
//!        Rotate by 90deg clockwise to get E, S and W, flip to get FN, FE,
//!        FS and FW. (think of a dial).
//!        Source: http://vlsicad.ucsd.edu/GSRC/bookshelf/Slots/Placement/plFormats.html
//! @note  This orientations are compatible with LEF/DEF and OpenAccess.

enum PhysicalOrientation : std::int8_t {
	/*! \brief Only used for Rsyn internal control.*/
	ORIENTATION_INVALID = -1,

	//! @brief North Orientation (R0)
	ORIENTATION_N = 0,
	//! @brief South orientation (R180)
	ORIENTATION_S = 1,
	//! @brief West orientation (R90)
	ORIENTATION_W = 2,
	//! @brief East orientation  (R270)
	ORIENTATION_E = 3,
	//! @brief Flipped-North orientation (MY)
	ORIENTATION_FN = 4,
	//! @brief Flipped-South orientation (MX)
	ORIENTATION_FS = 5,
	//! @brief Flipped-West orientation (MX90)
	ORIENTATION_FW = 6,
	//! @brief Flipped-East orientation (MY90)
	ORIENTATION_FE = 7,

	//! @brief Number of orientations
	NUM_PHY_ORIENTATION = 8
}; // end enum

enum BufferType {
	NON_INVERTING,
	INVERTING,
	ANY_BUFFER_TYPE
}; // end enum

enum TieType {
	TIE_LOW,
	TIE_HIGH,
	ANY_TIE_TYPE
}; // end enum

// Mateus @ 20190204: Refatoring Use
//! @brief Describes the usage of a pin or a net
//! @note Compatible with LEF/DEF standard 5.8
enum Use {
	UNKNOWN_USE = -1,
	SIGNAL,
	POWER,
	GROUND,
	CLOCK,
	TIEOFF,
	ANALOG,
	SCAN,
	RESET
};

} // end namespace

#endif