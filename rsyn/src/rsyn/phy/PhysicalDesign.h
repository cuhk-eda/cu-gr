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
 
#ifndef RSYN_PHYSICAL_DESIGN_H
#define RSYN_PHYSICAL_DESIGN_H

#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <iostream>
#include <stddef.h>
#include <algorithm>
#include <limits>

#include <Rsyn/Point>
#include <Rsyn/Rect>
#include <Rsyn/Polygon>

#include "rsyn/core/Rsyn.h"
#include "rsyn/util/Bounds.h"
#include "rsyn/util/FloatingPoint.h"
#include "rsyn/util/dbu.h"
#include "rsyn/util/Proxy.h"
#include "rsyn/phy/util/DefDescriptors.h"
#include "rsyn/phy/util/LefDescriptors.h"
#include "rsyn/phy/util/PhysicalTypes.h"
#include "rsyn/phy/util/PhysicalUtil.h"
#include "rsyn/phy/util/PhysicalTransform.h"
#include "rsyn/util/Exception.h"
#include "rsyn/util/Json.h"

namespace Rsyn {

// std::numeric_limts<PhysicalIndex>::max() reserved to invalid initialization object.
typedef std::uint32_t PhysicalIndex;
static PhysicalIndex INVALID_PHYSICAL_INDEX = std::numeric_limits<PhysicalIndex>::max();

class PhysicalObject;

class PhysicalRoutingPointData;
class PhysicalDieData;
class PhysicalGCellData;
class PhysicalLayerData;
class PhysicalSpacingData;
class PhysicalSpacingRuleData;
class PhysicalSpacingTableData;
class PhysicalSiteData;
class PhysicalRowData;
class PhysicalObstacleData;
class PhysicalPinLayerData;
class PhysicalPinGeometryData;
class PhysicalLibraryPinData;
class PhysicalLibraryCellData;
class PhysicalPinData;
class PhysicalNetData;
class PhysicalInstanceData;
class PhysicalRegionData;
class PhysicalGroupData;
class PhysicalViaData;
class ViaGeometryData;
class ViaRuleData;
class PhysicalSpecialWireData;
class PhysicalSpecialNetData;
class PhysicalTracksData;
class PhysicalRoutingGridData;
class LayerViaManagerData;
class PhysicalDesignData;

class PhysicalRoutingPoint;
class PhysicalDie;
class PhysicalGCell;
class PhysicalLayer;
class PhysicalSpacing;
class PhysicalSite;
class PhysicalRow;
class PhysicalObstacle;
class PhysicalPinLayer;
class PhysicalPinGeometry;
class PhysicalLibraryPin;
class PhysicalLibraryCell;
class PhysicalPin;
class PhysicalNet;
class PhysicalInstance;
class PhysicalPort;
class PhysicalCell;
class PhysicalModule;
class PhysicalRegion;
class PhysicalGroup;
class PhysicalVia;
class PhysicalViaGeometry;
class PhysicalViaRuleBase;
class PhysicalViaRule;
class PhysicalViaRuleGenerate;
class PhysicalSpecialWire;
class PhysicalSpecialNet;
class PhysicalTracks;
class PhysicalRoutingGrid;
class LayerViaManager;
class PhysicalDesign;

class PhysicalAttributeInitializer;
template<typename DefaultPhysicalValueType>
class PhysicalAttributeInitializerWithDefaultValue;

class PhysicalDesignObserver;
class PhysicalRouting;

} // end namespace 

// Object's Declarations (Proxies)
#include "rsyn/phy/obj/decl/PhysicalRoutingPoint.h"
#include "rsyn/phy/obj/decl/PhysicalDie.h"
#include "rsyn/phy/obj/decl/PhysicalGCell.h"
#include "rsyn/phy/obj/decl/PhysicalLayer.h"
#include "rsyn/phy/obj/decl/PhysicalSpacing.h"
#include "rsyn/phy/obj/decl/PhysicalSite.h"
#include "rsyn/phy/obj/decl/PhysicalRow.h"
#include "rsyn/phy/obj/decl/PhysicalPinLayer.h"
#include "rsyn/phy/obj/decl/PhysicalPinGeometry.h"
#include "rsyn/phy/obj/decl/PhysicalObstacle.h"
#include "rsyn/phy/obj/decl/PhysicalLibraryPin.h"
#include "rsyn/phy/obj/decl/PhysicalLibraryCell.h"
#include "rsyn/phy/obj/decl/PhysicalPin.h"
#include "rsyn/phy/obj/decl/PhysicalNet.h"
#include "rsyn/phy/obj/decl/PhysicalInstance.h"
#include "rsyn/phy/obj/decl/PhysicalCell.h"
#include "rsyn/phy/obj/decl/PhysicalModule.h"
#include "rsyn/phy/obj/decl/PhysicalPort.h"
#include "rsyn/phy/obj/decl/PhysicalRegion.h"
#include "rsyn/phy/obj/decl/PhysicalGroup.h"
#include "rsyn/phy/obj/decl/PhysicalVia.h"
#include "rsyn/phy/obj/decl/PhysicalViaGeometry.h"
#include "rsyn/phy/obj/decl/PhysicalViaRuleBase.h"
#include "rsyn/phy/obj/decl/PhysicalViaRule.h"
#include "rsyn/phy/obj/decl/PhysicalViaRuleGenerate.h"
#include "rsyn/phy/obj/decl/PhysicalSpecialWire.h"
#include "rsyn/phy/obj/decl/PhysicalSpecialNet.h"
#include "rsyn/phy/obj/decl/PhysicalTracks.h"
#include "rsyn/phy/obj/decl/PhysicalRoutingGrid.h"
#include "rsyn/phy/obj/decl/LayerViaManager.h"
#include "rsyn/phy/obj/decl/PhysicalDesign.h"

// Routing
#include "rsyn/phy/PhysicalRouting.h"

// Object's Data
#include "rsyn/phy/obj/data/PhysicalObject.h"
#include "rsyn/phy/obj/data/PhysicalRoutingPointData.h"
#include "rsyn/phy/obj/data/PhysicalDieData.h"
#include "rsyn/phy/obj/data/PhysicalGCellData.h"
#include "rsyn/phy/obj/data/PhysicalLayerData.h"
#include "rsyn/phy/obj/data/PhysicalSpacingData.h"
#include "rsyn/phy/obj/data/PhysicalSpacingRuleData.h"
#include "rsyn/phy/obj/data/PhysicalSpacingTableData.h"
#include "rsyn/phy/obj/data/PhysicalSiteData.h"
#include "rsyn/phy/obj/data/PhysicalRowData.h"
#include "rsyn/phy/obj/data/PhysicalPinLayerData.h"
#include "rsyn/phy/obj/data/PhysicalPinGeometryData.h"
#include "rsyn/phy/obj/data/PhysicalObstacleData.h"
#include "rsyn/phy/obj/data/PhysicalLibraryPinData.h"
#include "rsyn/phy/obj/data/PhysicalLibraryCellData.h"
#include "rsyn/phy/obj/data/PhysicalPinData.h"
#include "rsyn/phy/obj/data/PhysicalNetData.h"
#include "rsyn/phy/obj/data/PhysicalInstanceData.h"
#include "rsyn/phy/obj/data/PhysicalRegionData.h"
#include "rsyn/phy/obj/data/PhysicalGroupData.h"
#include "rsyn/phy/obj/data/PhysicalViaData.h"
#include "rsyn/phy/obj/data/ViaGeometryData.h"
#include "rsyn/phy/obj/data/ViaRuleData.h"
#include "rsyn/phy/obj/data/PhysicalSpecialWireData.h"
#include "rsyn/phy/obj/data/PhysicalSpecialNetData.h"
#include "rsyn/phy/obj/data/PhysicalTracksData.h"
#include "rsyn/phy/obj/data/PhysicalRoutingGridData.h"
#include "rsyn/phy/obj/data/LayerViaManagerData.h"
#include "rsyn/phy/obj/data/PhysicalDesign.h"

// Physical Infrastructure
#include "rsyn/phy/infra/PhysicalAttribute.h"
#include "rsyn/phy/infra/PhysicalObserver.h"

// Object's Implementations
#include "rsyn/phy/obj/impl/PhysicalRoutingPoint.h"
#include "rsyn/phy/obj/impl/PhysicalLayer.h"
#include "rsyn/phy/obj/impl/PhysicalDie.h"
#include "rsyn/phy/obj/impl/PhysicalGCell.h"
#include "rsyn/phy/obj/impl/PhysicalSpacing.h"
#include "rsyn/phy/obj/impl/PhysicalSite.h"
#include "rsyn/phy/obj/impl/PhysicalRow.h"
#include "rsyn/phy/obj/impl/PhysicalPinLayer.h"
#include "rsyn/phy/obj/impl/PhysicalPinGeometry.h"
#include "rsyn/phy/obj/impl/PhysicalObstacle.h"
#include "rsyn/phy/obj/impl/PhysicalLibraryPin.h"
#include "rsyn/phy/obj/impl/PhysicalLibraryCell.h"
#include "rsyn/phy/obj/impl/PhysicalPin.h"
#include "rsyn/phy/obj/impl/PhysicalNet.h"
#include "rsyn/phy/obj/impl/PhysicalInstance.h"
#include "rsyn/phy/obj/impl/PhysicalPort.h"
#include "rsyn/phy/obj/impl/PhysicalCell.h"
#include "rsyn/phy/obj/impl/PhysicalModule.h"
#include "rsyn/phy/obj/impl/PhysicalRegion.h"
#include "rsyn/phy/obj/impl/PhysicalGroup.h"
#include "rsyn/phy/obj/impl/PhysicalVia.h"
#include "rsyn/phy/obj/impl/PhysicalViaGeometry.h"
#include "rsyn/phy/obj/impl/PhysicalViaRuleBase.h"
#include "rsyn/phy/obj/impl/PhysicalViaRule.h"
#include "rsyn/phy/obj/impl/PhysicalViaRuleGenerate.h"
#include "rsyn/phy/obj/impl/PhysicalSpecialWire.h"
#include "rsyn/phy/obj/impl/PhysicalSpecialNet.h"
#include "rsyn/phy/obj/impl/PhysicalTracks.h"
#include "rsyn/phy/obj/impl/PhysicalRoutingGrid.h"
#include "rsyn/phy/obj/impl/LayerViaManager.h"
#include "rsyn/phy/obj/impl/PhysicalDesign.h"

#endif /* PHYSICALDESIGN_PHYSICALDESIGN__H */

