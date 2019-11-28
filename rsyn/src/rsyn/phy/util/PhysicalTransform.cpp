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
 
#include "PhysicalTransform.h"

namespace Rsyn {
const DBU PhysicalTransform::TRANSFORMATION_MATRIXES[NUM_PHY_ORIENTATION][2][2] = {
	// NORTH (R0)
	{{ 1,  0},             // | cos(0)  sin(0)|
	 { 0,  1}},            // |-sin(0)  cos(0)|

	// SOUTH (R180)
	{{-1,  0},             // | cos(-180)  sin(-180)|
	 { 0, -1}},            // |-sin(-180)  cos(-180)|

	// WEST (R90)
	{{ 0, -1},             // | cos(-90)  sin(-90)|
	 { 1,  0}},            // |-sin(-90)  cos(-90)|

	// EAST (R270)
	{{ 0,  1},             // | cos(-270)  sin(-270)|
	 {-1,  0}},            // |-sin(-270)  cos(-270)|

	// FLIPPED NORTH (MY)
	{{-1,  0},             // |-1  0|
	 { 0,  1}},            // | 0  1|

	// FLIPPED SOUTH (MX)
	{{ 1,  0},             // |1   0|
	 { 0, -1}},            // |0  -1|

	// FLIPPED WEST (MX90)
	{{ 0,  1},             // |1   0| | cos(90)  sin(90)|
	 { 1,  0}},            // |0  -1| |-sin(90)  cos(90)|

	// FLIPPED EAST (MY90)
	{{ 0, -1},             // |-1  0| | cos(90)  sin(90)|
	 {-1,  0}}             // | 0  1| |-sin(90)  cos(90)|
}; // end array


const DBU PhysicalTransform::TRANSLATION_MATRIXES[NUM_PHY_ORIENTATION][2][2] = {
	// NORTH (R0)
	{{ 0,  0},             // 0
	 { 0,  0}},            // 0

	// SOUTH (R180)
	{{ 1,  0},             // w
	 { 0,  1}},            // h

	// WEST (R90)
	{{ 0,  1},             // h
	 { 0,  0}},            // 0

	// EAST (R270)
	{{ 0,  0},             // 0
	 { 1,  0}},            // w

	// FLIPPED NORTH (MY)
	{{ 1,  0},             // w
	 { 0,  0}},            // 0

	// FLIPPED SOUTH (MX)
	{{ 0,  0},             // 0
	 { 0,  1}},            // h

	// FLIPPED WEST (MX90)
	{{ 0,  0},             // 0
	 { 0,  0}},            // 0

	// FLIPPED EAST (MY90)
	{{ 0,  1},             // h
	 { 1,  0}}             // w
}; // end array
} // end namespace