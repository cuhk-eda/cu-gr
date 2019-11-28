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
		
struct ArcData : ObjectData {
	ArcType type : 3;
	Pin from;
	Pin to;
	
	union {
		// Place holder for initialization...
		void * extra;
		
		// Meaningful only when this is arc belongs to a cell.
		LibraryArcData * libraryArcData;
		
		// Meaningful only when this is arc belongs to a net.
		NetData * netData;
	}; // end union

	ArcData() : 
		type(UNKNOWN_ARC_TYPE),
		extra(nullptr),
		from(nullptr), 
		to(nullptr) {
	} // end constructor
}; // end struct

} // end namespace