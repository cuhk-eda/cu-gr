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
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PhysicalSite.h
 * Author: jucemar
 *
 * Created on 12 de Setembro de 2016, 22:47
 */

#ifndef PHYSICALDESIGN_PHYSICALSITE_H
#define PHYSICALDESIGN_PHYSICALSITE_H

namespace Rsyn {

class PhysicalSite : public Proxy<PhysicalSiteData> {
	friend class PhysicalDesign;
	friend class PhysicalDesignData;
protected:
	//! @brief Constructs a Rsyn::PhysicalSite object with a pointer to Rsyn::PhysicalSiteData.
	PhysicalSite(PhysicalSiteData * data) : Proxy(data) {}
public:
	//! @brief Constructs a Rsyn::PhysicalSite object with a null pointer to Rsyn::PhysicalSiteData.
	PhysicalSite() : Proxy(nullptr) {}
	//! @brief Constructs a Rsyn::PhysicalSite object with a null pointer to Rsyn::PhysicalSiteData.
	PhysicalSite(std::nullptr_t) : Proxy(nullptr) {}
	//! @brief Returns the Rsyn::PhysicalSite name.
	std::string getName() const;
	//! @brief Returns the Rsyn::PhysicalSit class.
	//! @param Rsyn::PhysicalSiteClass may be 1) PAD or 2) CORE.
	Rsyn::PhysicalSiteClass getClass() const;
	//! @brief Returns the site size. In X is the width and in Y is the height.
	DBUxy getSize() const;
	//! @brief Returns the site width.
	DBU getWidth() const;
	//! @brief Returns the site heigth.
	DBU getHeight() const;
	//! @brief Returns site width if Dimension is X. Otherwise, returns the site height.
	DBU getLength(const Dimension dim) const;
	//! @brief Returns the site symmetry
	Rsyn::PhysicalSymmetry getSymmetry() const;
}; // end class

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALSITE_H */

