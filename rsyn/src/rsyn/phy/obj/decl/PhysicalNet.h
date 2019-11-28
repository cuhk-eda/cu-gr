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
 
/* 
 * File:   PhysicalNet.h
 * Author: jucemar
 *
 * Created on 15 de Setembro de 2016, 19:16
 */

#ifndef RSYN_PHYSICAL_NET_H
#define RSYN_PHYSICAL_NET_H

namespace Rsyn { 

class PhysicalNet : public Proxy<PhysicalNetData> {
	friend class PhysicalDesign;
protected:
	//! @brief Constructs a Rsyn::PhysicalNet object with a pointer to Rsyn::PhysicalNetData.
	PhysicalNet(PhysicalNetData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalNet object with a null pointer to Rsyn::PhysicalNetData.
	PhysicalNet() : Proxy(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalNet object with a null pointer to Rsyn::PhysicalNetData.
	PhysicalNet(std::nullptr_t) : Proxy(nullptr) {}

	//! @brief Returns the net associated to this physical net.
	Rsyn::Net getNet() const;
	//! @brief Returns the name of the physical net.
	std::string getName() const;
	//! @brief Returns the semi perimeter of the Bound Box from PhysicalNet.
	//! @details In X is the length of net Bound Box for abscissa and in Y is 
	//! the length of net Bound Box for ordinate. 
	DBUxy getHPWL() const;
	//! @brief Returns the length of the Bound Box from PhysicalNet in the given dimension.
	DBU getHPWL(const Dimension dim);
	//! @brief Returns a constant reference to the bound box of the PhysicalNet.
	//! @details Bound box units is DBU. Its boundary limits are defined by the two most
	//! distant each other pins for each dimension. 
	const Bounds & getBounds() const;
	//! @brief Returns the point coordinate of the net bound box. The points that 
	//! determine the Bounds are left-lower and upper-right.
	DBUxy getCoordinate(const Boundary bound) const ;
	//! @brief Returns the point coordinate for the given dimension of the net bound box.
	//! The points that determine the Bounds are left-lower and upper-right.
	DBU getCoordinate(const Boundary bound, const Dimension dim) const;
	//! @brief Return the PhysicalPin object that determines the boundary of PhysicalNet 
	//! in one of its demensions.
	Rsyn::Pin getPinBoundary(const Boundary bound, const Dimension dim) const;
	//! @brief Returns the physical routing of this net.
	const PhysicalRouting &getRouting() const;
	//! @brief Returns true if the net is routed (has a non-empty routing).
	bool isRouted() const;
}; // end class 

} // end namespace 

#endif