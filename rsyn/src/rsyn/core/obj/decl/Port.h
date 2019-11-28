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

//! @brief A proxy class representing a netlist port.
class Port : public Instance {
	
RSYN_FRIEND_OF_GENERIC_LIST_COLLECTION;

friend class RawPointer;
friend class Design;
friend class Instance;

private:	
	Port(InstanceData * data) : Instance(data) {}
public:

	//! @brief Default constructor.
	Port() : Instance(nullptr) {}

	//! @brief Assignment constructor to allow null values.
	Port(std::nullptr_t) : Instance(nullptr) {}	

	//! @brief Returns the pin inside the current level of hierarchy (module).
	//! @note  This pin belongs to the port instance.
	Pin getInnerPin() const;

	//! @brief Returns the pin in the parent hierarchy (module).
	//! @note  This pin does not belongs to the port instance. It belongs to the
	//!        parent module instance. That is the outer pin is only virtually
	//!        associated to a port.
	Pin getOuterPin() const; 

	//! @brief Returns the inner/outer pin related to an outer/inner pin.
	//!        If the outer pin is passed, returns the inner pin.
	//!        If the inner pin is passed, returns the outer pin.
	//!        If the pin does not match both inner and outer pins, returns
	//!        null.
	//! @note  The outer pin may be null if the inner pin belongs to
	//!        a port in the top level hierarchy.
	Pin getOtherPin(Rsyn::Pin pin) const;

	//! @brief Returns the direction of this port.
	//! @note  The direction of a port matches the direction of its inner pin.
	Direction getDirection() const;
}; // end class
	
} // end namespace