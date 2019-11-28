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
 
#include <Rsyn/Session>
#include "rsyn/phy/PhysicalService.h"

namespace Rsyn {

void PhysicalService::start(const Rsyn::Json &params) {
	Rsyn::Session session;
	
	clsDesign = session.getDesign();
	clsPhysicalDesign.initPhysicalDesign(clsDesign, params);
	// Observe changes in the design.
	clsDesign.registerObserver(this);
} // end method

// -----------------------------------------------------------------------------

void PhysicalService::stop() {
} // end method

// -----------------------------------------------------------------------------

void PhysicalService::onPostCellRemap(Rsyn::Cell cell, Rsyn::LibraryCell oldLibraryCell) {
	PhysicalDesignData *data = clsPhysicalDesign.data;

	PhysicalLibraryCellData &oldPhysicalLibraryCell =
			data->clsPhysicalLibraryCells[oldLibraryCell];
	PhysicalLibraryCellData &newPhysicalLibraryCell =
			data->clsPhysicalLibraryCells[cell.getLibraryCell()];
	PhysicalInstanceData &physicalCell = data->clsPhysicalInstances[cell];

	const DBU newWidth = newPhysicalLibraryCell.clsSize[X];
	const DBU newHeight = newPhysicalLibraryCell.clsSize[Y];

	const DBU oldWidth = oldPhysicalLibraryCell.clsSize[X];
	const DBU oldHeight = oldPhysicalLibraryCell.clsSize[Y];

	// Update physical cell size.
	physicalCell.clsInstance->clsBounds.setLength(X, newWidth);
	physicalCell.clsInstance->clsBounds.setLength(Y, newHeight);

	// Update area.
	const DBU oldArea = oldWidth * oldHeight;
	if (physicalCell.clsBlock)
		data->clsTotalAreas[PHYSICAL_BLOCK] -= oldArea;
	if (cell.isFixed()) {
		if (cell.isPort())
			data->clsTotalAreas[PHYSICAL_PORT] -= oldArea;
		else
			data->clsTotalAreas[PHYSICAL_FIXED] -= oldArea;
	} else {
		data->clsTotalAreas[PHYSICAL_MOVABLE] -= oldArea;
	} // end if-else

	const DBU newArea = newWidth * newHeight;
	if (physicalCell.clsBlock)
		data->clsTotalAreas[PHYSICAL_BLOCK] += newArea;
	if (cell.isFixed()) {
		if (cell.isPort())
			data->clsTotalAreas[PHYSICAL_PORT] += newArea;
		else
			data->clsTotalAreas[PHYSICAL_FIXED] += newArea;
	} else {
		data->clsTotalAreas[PHYSICAL_MOVABLE] += newArea;
	} // end if-else
} // end method

// -----------------------------------------------------------------------------

void PhysicalService::onPostInstanceCreate(Rsyn::Instance instance) {
	if (instance.getType() != Rsyn::CELL) {
		std::cout << "[WARNING] Created instance is not a cell.\n";
		return;		
	} // end if

	PhysicalDesignData *data = clsPhysicalDesign.data;
	
	PhysicalLibraryCellData &physicalLibraryCell =
			data->clsPhysicalLibraryCells[instance.asCell().getLibraryCell()];	

	PhysicalInstanceData &physicalCell = data->clsPhysicalInstances[instance.asCell()];
	physicalCell.clsInstance = instance;
	Rsyn::InstanceTag tag = clsDesign.getTag(instance);
	tag.setFixed(false);
	tag.setMacroBlock(!strcmp(physicalLibraryCell.clsMacro->macroClass(), "BLOCK"));
	physicalCell.clsHasLayerBounds = false;
	physicalCell.clsPlaced = true;
	physicalCell.clsBlock = false;
	physicalCell.clsInstance->clsOrientation = PhysicalOrientation::ORIENTATION_N;
	
	const DBUxy dieOrigin = data->clsPhysicalDie.clsBounds[LOWER];
	physicalCell.clsInitialPos = DBUxy(dieOrigin);
	
	const DBU width = physicalLibraryCell.clsSize[X];
	const DBU height = physicalLibraryCell.clsSize[Y];
	
	physicalCell.clsInstance->clsBounds.updatePoints(
		dieOrigin.x, dieOrigin.y, dieOrigin.x + width, dieOrigin.y + height);
	
	DBU area = width * height;
	data->clsTotalAreas[PHYSICAL_MOVABLE] += area;
} // end method
} // end namespace
