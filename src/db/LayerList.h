#pragma once

#include "CutLayer.h"
#include "MetalLayer.h"
#include "RsynService.h"

namespace db {

class LayerList {
public:
    void init();

    // Check whether a geo primitive is valid
    bool isValid(const GridPoint& gridPt) const;
    bool isValid(const GridBoxOnLayer& gridBox) const;
    bool isValid(const ViaBox& viaBox) const;

    // Get metal rectangle (diff-net) forbidding region
    // 1. for pin taps
    BoxOnLayer getMetalRectForbidRegion(const BoxOnLayer& metalRect, AggrParaRunSpace aggr) const;
    // 2. for via forbid regions
    vector<utils::BoxT<DBU>> getAccurateMetalRectForbidRegions(const BoxOnLayer& metalRect) const;

    // Expand box by several pitches
    void expandBox(BoxOnLayer& box, int numPitchToExtend) const;
    void expandBox(BoxOnLayer& box, int numPitchToExtend, int dir) const;

    // NOTE: the functions below assume valid input geo primitive(s)

    // Search by location
    utils::IntervalT<int> getSurroundingTrack(int layerIdx, DBU loc) const;
    utils::IntervalT<int> getSurroundingCrossPoint(int layerIdx, DBU loc) const;
    GridBoxOnLayer getSurroundingGrid(int layerIdx, utils::PointT<DBU> loc) const;

    // Search by location range
    // input: layer (a valid one is assumed), location range [min, max] (inclusive)
    // output: index range of Track/CrossPoint
    // note: if out of range, assign the nearest endpoint
    utils::IntervalT<int> rangeSearchTrack(int layerIdx,
                                           const utils::IntervalT<DBU>& locRange,
                                           bool includeBound = true) const;
    utils::IntervalT<int> rangeSearchCrossPoint(int layerIdx,
                                                const utils::IntervalT<DBU>& locRange,
                                                bool includeBound = true) const;
    GridBoxOnLayer rangeSearch(const BoxOnLayer& box, bool includeBound = true) const;

    // Get (x, y) location of GridPoint/GridBoxOnLayer
    utils::PointT<DBU> getLoc(const GridPoint& gridPt) const;
    BoxOnLayer getLoc(const GridBoxOnLayer& gridBox) const;
    std::pair<utils::PointT<DBU>, utils::PointT<DBU>> getLoc(const GridEdge& edge) const;

    // Find the upper/lower GridPoint/GridBoxOnLayer of the current GridPoint/GridBoxOnLayer
    // note: 1. valid layer is assumed, 2. upper/lower GridPoint/GridBoxOnLayer may not exist.
    GridPoint getUpper(const GridPoint& cur) const;
    GridPoint getLower(const GridPoint& cur) const;
    GridBoxOnLayer getUpper(const GridBoxOnLayer& cur) const;
    GridBoxOnLayer getLower(const GridBoxOnLayer& cur) const;

    // Get ViaBox from the intersection of two BoxOnLayer (in neighboring layers)
    // note: 1. should be on neighboring layers, 2. ViaBox may be empty.
    ViaBox getViaBoxBetween(const BoxOnLayer& lower, const BoxOnLayer& upper);
    ViaBox getViaBoxBetween(const GridBoxOnLayer& lower, const GridBoxOnLayer& upper) {
        return getViaBoxBetween(getLoc(lower), getLoc(upper));
    }
    bool isConnected(const GridBoxOnLayer& lhs, const GridBoxOnLayer& rhs);
    bool isAdjacent(const GridBoxOnLayer& lhs, const GridBoxOnLayer& rhs);

    Dimension getLayerDir(int layerIdx) const { return layers[layerIdx].direction; }
    const MetalLayer& getLayer(int layerIdx) const { return layers[layerIdx]; }
    const CutLayer& getCutLayer(int cutLayerIdx) const { return cutLayers[cutLayerIdx]; }
    unsigned getLayerNum() const noexcept { return layers.size(); }

    // Merge LUTs
    void mergeLUT(vector<vector<bool>>& lhs, const vector<vector<bool>>& rhs);
    vector<vector<bool>> mergeLUTs(const vector<vector<vector<bool>>>& LUTs);
    vector<vector<vector<bool>>> mergeLUTsCP(const vector<vector<vector<vector<bool>>>>& LUTs);

protected:
    vector<MetalLayer> layers;
    vector<CutLayer> cutLayers;

    int numGridPoints;
    DBU totalTrackLength;
    int numVias;

    void initCrossPoints();
    static void initOppLUT(const vector<vector<vector<bool>>>& ori, vector<vector<vector<bool>>>& opp);
    void initViaWire(const int layerIdx, const utils::BoxT<DBU>& viaMetal, vector<vector<vector<bool>>>& viaWireLUT);
    void initSameLayerViaConfLUT(const int layerIdx,
                                 ViaType& viaT1,
                                 ViaType& viaT2,
                                 vector<vector<bool>>& viaCut,
                                 vector<vector<bool>>& viaMetal,
                                 vector<vector<int>>& viaMetalNum);
    void initDiffLayerViaConfLUT(const int layerIdx,
                                 ViaType& viaT1,
                                 ViaType& viaT2,
                                 vector<vector<vector<bool>>>& viaBotVia,
                                 vector<vector<vector<bool>>>& viaTopVia);
    void initViaConfLUT();
    void initViaForbidRegions();
    void print();
    void writeDefConflictLUTs(const std::string& debugFileName) const;
};

}  //   namespace db
