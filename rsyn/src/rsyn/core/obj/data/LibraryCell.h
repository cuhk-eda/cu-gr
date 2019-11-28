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

struct LibraryCellTagData {
	LogicTypeTag logicType : 2;
	BufferTypeTag bufferType : 2;
	TieTypeTag tieType : 2;

	LibraryCellTagData() :
		logicType(LOGIC_TYPE_TAG_NOT_SPECIFIED),
		bufferType(BUFFER_TYPE_TAG_NOT_SPECIFIED),
		tieType(TIE_TYPE_TAG_NOT_SPECIFIED) {
	} // end constructor
}; // end struct

// -----------------------------------------------------------------------------

struct LibraryCellData : ObjectData {
	Design design;
	std::string name;
	std::vector<LibraryPin> pins;
	std::vector<LibraryArc> arcs;
	DBUxy size;

	// User flags.
	LibraryCellTagData tag;

	// TODO: use array
	int numInputPins;
	int numOutputPins;
	int numInOutPins;

	LibraryCellData() : 
		design(nullptr),
		numInputPins(0),
		numOutputPins(0), 
		numInOutPins(0),
		size(0, 0) {
	} // constructor
}; // end struct

} // end namespace