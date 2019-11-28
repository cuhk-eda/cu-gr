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
 
namespace Rsyn {

struct LibraryPinData : ObjectData {
	Design design;
	LibraryCell lcell;
	std::string name;
	Direction direction;
	// Mateus @ 20180917: Adding PinUse
	Use pinUse;
	int index;

	LibraryPinData() : 
		design(nullptr),
		lcell(nullptr),
		direction(UNKNOWN_DIRECTION),
		pinUse(UNKNOWN_USE),
		index(-1) {
	} // end constructor
}; // end struct

} // end namespace