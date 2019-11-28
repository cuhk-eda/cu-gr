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

struct LibraryArcData : ObjectData {
	Design design;
	int index; // TODO: index may be confusing as objects have a field called id 
	
	LibraryCell lcell;
	LibraryPin from;
	LibraryPin to;

	LibraryArcData() : 
		design(nullptr),
		index(-1), 
		lcell(nullptr),
		from(nullptr), 
		to(nullptr) {
	} // end class
}; // end struct

} // end namespace