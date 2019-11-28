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
Design 
LibraryPin::getDesign() {
	return data->design;
} // end method

// -----------------------------------------------------------------------------

inline
const Design 
LibraryPin::getDesign() const {
	return data->design;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
LibraryPin::getName() const {
	return data? data->name : NullName;
} // end method

// -----------------------------------------------------------------------------

inline
std::string
LibraryPin::getHierarchicalName(const std::string & separator) const {
	return data ? data->lcell.getName() + separator + data->name : NullName;
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
LibraryPin::getDirectionName() const {
	return Global::getDirectionName(getDirection());
} // end method

// -----------------------------------------------------------------------------

inline
const std::string &
LibraryPin::getLibraryCellName() const {
	return getLibraryCell().getName();
} // end method

// -----------------------------------------------------------------------------

inline
Direction 
LibraryPin::getDirection() const {
	return data->direction;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell 
LibraryPin::getLibraryCell() const {
	return data->lcell;
} // end method

// -----------------------------------------------------------------------------

inline
int 
LibraryPin::getIndex() const {
	return data->index;
} // end method

// -----------------------------------------------------------------------------

inline
bool 
LibraryPin::isInput() const {
	return getDirection() == Rsyn::IN;
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryPin::isOutput() const {
	return getDirection() == Rsyn::OUT;
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryPin::isBidirectional() const {
	return getDirection() == Rsyn::BIDIRECTIONAL;
} // end method

// -----------------------------------------------------------------------------

inline
Use
LibraryPin::getUse() const {
	return data->pinUse;
} // end method

// -----------------------------------------------------------------------------

inline
bool
LibraryPin::isPowerOrGround() const {
	return (getUse() == POWER || getUse() == GROUND);
} // end method

} // end namespace