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

namespace Rsyn {

// -----------------------------------------------------------------------------

inline Rsyn::Cell PhysicalCell::getCell() const {
	return data->clsInstance.asCell();
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalCell::isFixed() const {
	return data->clsInstance.isFixed();
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalCell::isMacroBlock() const {
	return data->clsInstance.isMacroBlock();
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalCell::isPlaced() const {
	return data->clsPlaced;
} // end method 

// -----------------------------------------------------------------------------

inline DBUxy PhysicalCell::getInitialPosition() const {
	return data->clsInitialPos;
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getInitialPosition(const Dimension dim) const {
	return data->clsInitialPos[dim];
} // end method

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacement() const {
	const DBUxy pos = getPosition();
	const DBUxy initial = getInitialPosition();
	return std::abs(pos[X] - initial[X]) + std::abs(pos[Y] - initial[Y]);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacement(const Dimension dim) const {
	const DBU pos = getPosition(dim);
	const DBU initial = getInitialPosition(dim);
	return std::abs(initial - pos);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacement(const DBUxy pos) const {
	const DBUxy lower = getPosition();
	return std::abs(lower[X] - pos[X]) + std::abs(lower[Y] - pos[Y]);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacement(const DBU pos, const Dimension dim) const {
	const DBU lower = getPosition(dim);
	return std::abs(lower - pos);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacementFromCurrentPosition(const DBUxy pos) const {
	const DBUxy lower = getPosition();
	return std::abs(lower[X] - pos[X]) + std::abs(lower[Y] - pos[Y]);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacementFromCurrentPosition(const DBU pos, const Dimension dim) const {
	const DBU lower = getPosition(dim);
	return std::abs(lower - pos);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacementFromInitialPosition(const DBUxy pos) const {
	const DBUxy lower = getInitialPosition();
	return std::abs(lower[X] - pos[X]) + std::abs(lower[Y] - pos[Y]);
} // end method 

// -----------------------------------------------------------------------------

inline DBU PhysicalCell::getDisplacementFromInitialPosition(const DBU pos, const Dimension dim) const {
	const DBU lower = getInitialPosition(dim);
	return std::abs(lower - pos);
} // end method 

// -----------------------------------------------------------------------------

inline PhysicalTransform PhysicalCell::getTransform(const bool origin) const {
	return getInstance().getTransform(origin);
} // end method

// -----------------------------------------------------------------------------

inline bool PhysicalCell::hasLayerBounds() const {
	return data->clsHasLayerBounds;
} // end method 

// -----------------------------------------------------------------------------

} // end namespace 