#pragma once

#include "global.h"

namespace db {

//  BoxOnLayer
//  A box on a certain layer: primitive for route guide and pin acesss box

class BoxOnLayer : public utils::BoxT<DBU> {
public:
    int layerIdx;

    //  constructors
    template <typename... Args>
    BoxOnLayer(int layerIndex = -1, Args... params) : layerIdx(layerIndex), utils::BoxT<DBU>(params...) {}

    // inherit setters from utils::BoxT in batch
    template <typename... Args>
    void Set(int layerIndex = -1, Args... params) {
        layerIdx = layerIndex;
        utils::BoxT<DBU>::Set(params...);
    }

    bool isConnected(const BoxOnLayer& rhs) const;

    friend ostream& operator<<(ostream& os, const BoxOnLayer& box);
};

// helper
utils::BoxT<DBU> getBoxFromRsynBounds(const Bounds& bounds);
utils::BoxT<DBU> getBoxFromRsynGeometries(const vector<Rsyn::PhysicalViaGeometry>& geos);

// GridPoint

class GridPoint {
public:
    int layerIdx;
    int trackIdx;
    int crossPointIdx;

    GridPoint(int layerIndex = -1, int trackIndex = -1, int crossPointIndex = -1)
        : layerIdx(layerIndex), trackIdx(trackIndex), crossPointIdx(crossPointIndex) {}

    bool operator==(const GridPoint& rhs) const;
    bool operator!=(const GridPoint& rhs) const;

    friend ostream& operator<<(ostream& os, const GridPoint& gp);
};

}  // namespace db

namespace std {

// hash function for GridPoint
template <>
struct hash<db::GridPoint> {
    std::size_t operator()(const db::GridPoint& gp) const {
        return (std::hash<int>()(gp.layerIdx) ^ std::hash<int>()(gp.trackIdx) ^ std::hash<int>()(gp.crossPointIdx));
    }
};

}  // namespace std

namespace db {

// GridEdge

class GridEdge {
public:
    GridPoint u, v;

    GridEdge(const GridPoint& nodeU, const GridPoint& nodeV) : u(nodeU), v(nodeV) {}

    // two types of GridEdge: 1. via, 2. track segment
    bool isVia() const;
    const GridPoint& lowerGridPoint() const { return u.layerIdx <= v.layerIdx ? u : v; }
    bool isTrackSegment() const;
    bool isWrongWaySegment() const;

    bool operator==(const GridEdge& rhs) const;

    friend ostream& operator<<(ostream& os, const GridEdge& edge);
};

class TrackSegment {
public:
    int layerIdx;
    int trackIdx;
    utils::IntervalT<int> crossPointRange;

    // assume edge.isTrackSegment()
    TrackSegment(const GridEdge edge)
        : layerIdx(edge.u.layerIdx),
          trackIdx(edge.u.trackIdx),
          crossPointRange(edge.u.crossPointIdx, edge.v.crossPointIdx) {
        if (!crossPointRange.IsValid()) std::swap(crossPointRange.low, crossPointRange.high);
    }

    TrackSegment(int layerIndex, int trackIndex, const utils::IntervalT<int>& crossPointIndexRange)
        : layerIdx(layerIndex), trackIdx(trackIndex), crossPointRange(crossPointIndexRange) {}

    friend ostream& operator<<(ostream& os, const TrackSegment& ts);
};

// GridBoxOnLayer

class GridBoxOnLayer {
public:
    int layerIdx;
    utils::IntervalT<int> trackRange;
    utils::IntervalT<int> crossPointRange;

    GridBoxOnLayer() : layerIdx(-1) {}  // default to be invalid

    GridBoxOnLayer(int layerIndex,
                   const utils::IntervalT<int>& trackIdxRange,
                   const utils::IntervalT<int>& crossPointIdxRange)
        : layerIdx(layerIndex), trackRange(trackIdxRange), crossPointRange(crossPointIdxRange) {}

    bool includePoint(const GridPoint& point) const {
        return layerIdx == point.layerIdx && trackRange.Contain(point.trackIdx) &&
               crossPointRange.Contain(point.crossPointIdx);
    }

    // slice polygons along sliceDir
    // sliceDir: 0 for x/vertical, 1 for y/horizontal
    // assume boxes are on the same layer
    static void sliceGridPolygons(vector<GridBoxOnLayer>& boxes);
    const utils::IntervalT<int>& operator[](unsigned i) const {
        assert(i == 0 || i == 1);
        return (i == 0) ? trackRange : crossPointRange;
    }
    utils::IntervalT<int>& operator[](unsigned i) {
        assert(i == 0 || i == 1);
        return (i == 0) ? trackRange : crossPointRange;
    }

    bool operator==(const GridBoxOnLayer& rhs) const;

    friend ostream& operator<<(ostream& os, const GridBoxOnLayer& gb);
};

// ViaBox

class ViaBox {
public:
    GridBoxOnLayer lower, upper;

    ViaBox() {}  // default to be invalid

    ViaBox(const GridBoxOnLayer& lowerGridBox, const GridBoxOnLayer& upperGridBox)
        : lower(lowerGridBox), upper(upperGridBox) {}

    friend ostream& operator<<(ostream& os, const ViaBox& vb);
};

}  // namespace db