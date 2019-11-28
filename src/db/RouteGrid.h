#pragma once

#include "LayerList.h"
#include "Net.h"
#include "Setting.h"

namespace db {

class ViaData;

using CostT = double;

struct mutex_wrapper : std::mutex {
    mutex_wrapper() = default;
    mutex_wrapper(mutex_wrapper const&) noexcept : std::mutex() {}
    bool operator==(mutex_wrapper const& other) noexcept { return this == &other; }
};

// net index
// a valid net idx >= 0
const int OBS_NET_IDX = -1;   // for obstacles
const int NULL_NET_IDX = -2;  // for neither net nor obstacle

class RouteGrid : public LayerList {
public:
    using ViaMapT = vector<vector<std::multimap<int, int>>>;
    using NDViaMapT = std::unordered_map<GridPoint, const ViaType*>;

    void init();
    void clear();
    void setUnitVioCost(double discount = 1.0);

    // Query fixed metals
    vector<std::pair<utils::BoxT<DBU>, int>> getOvlpBoxes(const BoxOnLayer& box, int idx, const RTrees& rtree) const;
    vector<std::pair<utils::BoxT<DBU>, int>> getOvlpFixedMetals(const BoxOnLayer& box, int netIdx) const;

    int getFixedMetalVio(const BoxOnLayer& box, int netIdx) const;
    DBU getOvlpFixedMetalArea(const BoxOnLayer& box, int netIdx) const;

    void markFixedMetalBatch(vector<std::pair<BoxOnLayer, int>>& fixedMetalVec, int beginIdx, int endIdx);
    const RTree& getFixedMetals(int layerIdx) const { return fixedMetals[layerIdx]; }

    DBU getUnitViaCost() const { return unitViaCost; }
    DBU getUnitShortCost(int layerIdx) const { return unitShortVioCostDiscounted[layerIdx]; }

protected:
    // Unit cost
    // in contest metric
    CostT unitWireCostRaw;                                       // for each DBU
    CostT unitViaCostRaw;                                        // for each occurrence
    CostT unitShortVioCostRaw;                                   // for each area of m2Pitch * m2Pitch
    vector<CostT> unitShortVioCost, unitShortVioCostDiscounted;  // ceoff (no unit), a M2 short will charged by
                                                                 // (shortLength * unitWireCostRaw) * unitShortVioCost
    // in DBU
    CostT unitViaCost;  // for each occurrence

    // Fixed metals (e.g., cell pins, blockage)
    RTrees fixedMetals;  // box -> netIdx
};

}  //   namespace db
