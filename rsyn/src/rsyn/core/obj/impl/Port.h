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

// -----------------------------------------------------------------------------

inline
Pin 
Port::getInnerPin() const {
	// Ports have just one pin, which is the inner pin...
	return data->pins[0];
} // end method

// -----------------------------------------------------------------------------

inline
Pin 
Port::getOuterPin() const {
	return Pin(data->outerPin);
} // end method

// -----------------------------------------------------------------------------

inline
Pin 
Port::getOtherPin(Pin pin) const {
	Rsyn::Pin inner = getInnerPin();
	Rsyn::Pin outer = getOuterPin();
	if (inner == pin) return outer;
	if (outer == pin) return inner;
	return nullptr;
} // end method

// -----------------------------------------------------------------------------

inline
Direction
Port::getDirection() const {
	return Global::getReverseDirection(getInnerPin().getDirection());
} // end method
	
} // end namespace