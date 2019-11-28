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

inline
const std::string &
Cell::getLibraryCellName() const {
	return data->lcell->name;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell
Cell::getLibraryCell() const {
	return data->lcell;
} // end method

// -----------------------------------------------------------------------------

inline
Pin
Cell::getPinByLibraryPin(const LibraryPin lpin) const {
	return data->pins[lpin.data->index];
} // end method
	
// -----------------------------------------------------------------------------

inline
void 
Cell::remap(LibraryCell libraryCell) {
	getDesign().remap(*this, libraryCell);
} // end method

// -----------------------------------------------------------------------------

inline
void 
Cell::remap(const std::string &libraryCellName) {
	getDesign().remap(*this, libraryCellName);
} // end method

} // end namespace