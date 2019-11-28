#pragma once

#include "GeoPrimitive.h"

namespace db {

class ViaType {
public:
    bool hasMultiCut = false;
    utils::BoxT<DBU> bot;  // box on bottom metal layer
    utils::BoxT<DBU> top;  // box on top metal layer
    utils::BoxT<DBU> cut;  // box on cut layer
    std::string name;
    int idx;

    vector<utils::BoxT<DBU>> botForbidRegions;
    vector<utils::BoxT<DBU>> topForbidRegions;

    // via-wire conflict (crossPointIdx, trackIdx, crossPointIdx)
    vector<vector<vector<bool>>> viaBotWire;
    vector<vector<vector<bool>>> viaTopWire;

    // same-layer via-via conflict (viaTypeIdx, lowerTrackIdx, upperTrackIdx)
    // TODO: remove allViaMetal, rename allViaMetalNum to allViaMetal
    vector<vector<vector<bool>>> allViaCut;      // due to cut spacing
    vector<vector<vector<bool>>> allViaMetal;    // due to metal spacing
    vector<vector<vector<int>>> allViaMetalNum;  // due to metal spacing, integer version

    // cross-layer via-via conflict (viaTypeIdx, crossPointIdx, trackIdx, crossPointIdx)
    vector<vector<vector<vector<bool>>>> allViaBotVia;
    vector<vector<vector<vector<bool>>>> allViaTopVia;

    // merged LUTs
    vector<vector<bool>> mergedAllViaMetal;
    vector<vector<vector<bool>>> mergedAllViaBotVia;
    vector<vector<vector<bool>>> mergedAllViaTopVia;

    ViaType() {}
    ViaType(Rsyn::PhysicalVia rsynVia);

    // alphabetical score tuple (belowWidth, aboveWidth, belowLength, aboveLength)
    std::tuple<DBU, DBU, DBU, DBU> getDefaultScore(const Dimension botDim, const Dimension topDim) const;

    // shifted bot/top metal
    utils::BoxT<DBU> getShiftedBotMetal(const utils::PointT<DBU>& viaPos) const;
    utils::BoxT<DBU> getShiftedTopMetal(const utils::PointT<DBU>& viaPos) const;
};

class CutLayer {
public:
    CutLayer(const Rsyn::PhysicalLayer& rsynLayer,
             const vector<Rsyn::PhysicalVia>& rsynVias,
             const Dimension botDim,
             const Dimension topDim,
             const DBU libDBU);

    // Basic infomation
    std::string name;
    int idx;  // layerIdx (consistent with Rsyn::xxx::getRelativeIndex())

    // Design rules
    DBU width = 0;
    DBU spacing = 0;

    // Via types
    vector<ViaType> allViaTypes;
    const ViaType& defaultViaType() const { return allViaTypes[0]; }
    bool isDefaultViaType(const ViaType& viaType) const { return viaType.idx == defaultViaType().idx; }
    utils::BoxT<DBU> topMaxForbidRegion;
    utils::BoxT<DBU> botMaxForbidRegion;

    // Via conflict lookup table (true means "not available" / with conflict)
    // 1. same-layer via-via conflict (lowerTrackIdx, upperTrackIdx)
    //  due to cut spacing
    const vector<vector<bool>>& viaCut() const { return defaultViaType().allViaCut[0]; }
    //  due to metal spacing
    const vector<vector<bool>>& viaMetal() const { return defaultViaType().allViaMetal[0]; }
    const vector<vector<int>>& viaMetalNum() const { return defaultViaType().allViaMetalNum[0]; }
    // 2. via-via conflict (crossPointIdx, trackIdx, crossPointIdx)
    const vector<vector<vector<bool>>>& viaBotVia() const { return defaultViaType().allViaBotVia[0]; }
    const vector<vector<vector<bool>>>& viaTopVia() const { return defaultViaType().allViaTopVia[0]; }
    // 3. via-wire conflict (crossPointIdx, trackIdx, crossPointIdx)
    const vector<vector<vector<bool>>>& viaBotWire() const { return defaultViaType().viaBotWire; }
    const vector<vector<vector<bool>>>& viaTopWire() const { return defaultViaType().viaTopWire; }

    ostream& printBasics(ostream& os) const;
    ostream& printDesignRules(ostream& os) const;
    ostream& printViaOccupancyLUT(ostream& os) const;
    friend ostream& operator<<(ostream& os, const CutLayer& layer);
};

}  // namespace db
