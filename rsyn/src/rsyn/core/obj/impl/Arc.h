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
Arc::getDesign() {
	return getFromPin().getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
const Design 
Arc::getDesign() const {
	return getFromPin().getDesign();
} // end method

// -----------------------------------------------------------------------------

inline
ArcType 
Arc::getType() const {
	return data->type;
} // end method

// -----------------------------------------------------------------------------

inline
Instance 
Arc::getInstance() const {
	return data->type == INSTANCE_ARC? data->from.getInstance() : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Net 
Arc::getNet() const {
	return data->type == NET_ARC? data->netData : nullptr;
} // end method
	
// -----------------------------------------------------------------------------
	
inline
LibraryArc
Arc::getLibraryArc() const {
	return getType() == INSTANCE_ARC? data->libraryArcData : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
LibraryCell
Arc::getLibraryCell() const {
	Rsyn::LibraryArc larc = getLibraryArc();
	return larc? larc.getLibraryCell() : nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Pin 
Arc::getFromPin() const {
	return data->from;
} // end method

// -----------------------------------------------------------------------------	
	
inline
Pin 
Arc::getToPin() const {
	return data->to;
} // end method

// -----------------------------------------------------------------------------

inline
std::string
Arc::getName() const {
	return getFromName() + "->" + getToName();
} // end method

// -----------------------------------------------------------------------------

inline
std::string
Arc::getFullName() const {
	return getType() == NET_ARC?
		(getFromPin().getFullName() + "->" + getToPin().getFullName()) :
		(getInstance().getName() + ":" + getName());
} // end method

// -----------------------------------------------------------------------------

inline
const std::string
Arc::getFromName() const {
	return getFromPin().getName();
} // end method

// -----------------------------------------------------------------------------

inline
const std::string
Arc::getToName() const {
	return getToPin().getName();
} // end method

// -----------------------------------------------------------------------------

inline
Net
Arc::getFromNet() const {
	return getFromPin().getNet();
} // end method

// -----------------------------------------------------------------------------

inline
Net
Arc::getToNet() const {
	return getToPin().getNet();
} // end method

// -----------------------------------------------------------------------------

inline
int
Arc::getIndex() const {
	return getLibraryArc()? getLibraryArc().getIndex() : -1;
} // end method

} // end namespace