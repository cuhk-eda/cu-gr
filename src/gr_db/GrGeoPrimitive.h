#pragma once

#include "global.h"
#include "db/Database.h"

namespace gr {
// GrPoint
class GrPoint : public utils::PointT<int> {
public:
    int layerIdx;

    GrPoint(int layerIndex = -1, int xx = -1, int yy = -1) : PointT<int>(xx, yy), layerIdx(layerIndex) {}

    int getPrefIdx() const { return database.getLayerDir(layerIdx) == X ? x : y; }

    bool operator==(const GrPoint& rhs) const;
    bool operator!=(const GrPoint& rhs) const;

    friend ostream& operator<<(ostream& os, const GrPoint& gp);
};

// PointOnLayer
class PointOnLayer : public utils::PointT<int> {
public:
    int layerIdx;

    PointOnLayer(int layerIndex = -1, int xx = -1, int yy = -1) : PointT<int>(xx, yy), layerIdx(layerIndex) {}
    PointOnLayer(const GrPoint& point) : PointT<int>(point[X], point[Y]), layerIdx(point.layerIdx) {}

    bool operator==(const PointOnLayer& rhs) const;
    bool operator!=(const PointOnLayer& rhs) const;

    friend ostream& operator<<(ostream& os, const PointOnLayer& gp);
};

// GrEdge
class GrEdge {
public:
    GrPoint u, v;

    GrEdge(const GrPoint& nodeU, const GrPoint& nodeV) {
        // ensure u is always smaller than v
        if (nodeU[0] <= nodeV[0] && nodeU[1] <= nodeV[1]) {
            u = nodeU;
            v = nodeV;
        } else {
            u = nodeV;
            v = nodeU;
        }
    }

    GrEdge(int layerIdx, int g, int cp) {
        auto dir = database.getLayerDir(layerIdx);
        if (dir == X) {
            u = GrPoint(layerIdx, g, cp);
            v = GrPoint(layerIdx, g, cp + 1);
        } else {
            u = GrPoint(layerIdx, cp, g);
            v = GrPoint(layerIdx, cp + 1, g);
        }
    }

    GrEdge(int layerIdx, int g, int lo_cp, int hi_cp) {
        auto dir = database.getLayerDir(layerIdx);
        if (dir == X) {
            u = GrPoint(layerIdx, g, lo_cp);
            v = GrPoint(layerIdx, g, hi_cp);
        } else {
            u = GrPoint(layerIdx, lo_cp, g);
            v = GrPoint(layerIdx, hi_cp, g);
        }
    }

    int getLayerIdx() const {
        if (isVia())
            return -1;
        else
            return u.layerIdx;
    }
    int getCutLayerIdx() const {
        if (isVia())
            return lowerGrPoint().layerIdx;
        else
            return -1;
    }
    DBU getGrLen() const {
        DBU len = 0;
        if (!isVia()) len = v[X] - u[X] + v[Y] - u[Y];
        return len;
    }

    // two types of GridEdge: 1. via, 2. segment
    bool isVia() const { return u.layerIdx != v.layerIdx; }
    const GrPoint& lowerGrPoint() const { return u.layerIdx <= v.layerIdx ? u : v; }
    const GrPoint& upperGrPoint() const { return u.layerIdx > v.layerIdx ? u : v; }

    bool operator==(const GrEdge& rhs) const;

    friend ostream& operator<<(ostream& os, const GrEdge& edge);
};

// GrBoxOnLayer
class GrBoxOnLayer : public utils::BoxT<int> {
public:
    int layerIdx;

    GrBoxOnLayer() : layerIdx(-1) {}  // default to be invalid

    GrBoxOnLayer(int layerIndex, const utils::IntervalT<int>& xx, const utils::IntervalT<int>& yy)
        : layerIdx(layerIndex), utils::BoxT<int>(xx, yy) {}

    bool includePoint(const GrPoint& point, bool ignoreLayer = false) const {
        return (ignoreLayer || layerIdx == point.layerIdx) && x.Contain(point[X]) && y.Contain(point[Y]);
    }
    // slice polygons along pref direction
    // assume boxes are on the same layer
    static void sliceGrPolygons(vector<GrBoxOnLayer>& boxes, bool mergeAdj);

    bool operator==(const GrBoxOnLayer& rhs) const;
    friend ostream& operator<<(ostream& os, const GrBoxOnLayer& gb);
};

}  // namespace gr

namespace std {

// hash function for GrPoint
template <>
struct hash<gr::GrPoint> {
    std::size_t operator()(const gr::GrPoint& gp) const {
        return (std::hash<int>()(gp.layerIdx) ^ std::hash<int>()(gp.x) ^ std::hash<int>()(gp.y));
    }
};

// hash for gr::GrEdge
template <>
struct hash<gr::GrEdge> {
    std::size_t operator()(const gr::GrEdge edge) const {
        // return 0;
        return std::hash<int>()(edge.u.layerIdx) ^ std::hash<int>()(edge.u.x) ^ std::hash<int>()(edge.u.y) ^
               std::hash<int>()(edge.v.x) ^ std::hash<int>()(edge.v.y);
    }
};

// hash function for PointOnLayer
template <>
struct hash<gr::PointOnLayer> {
    std::size_t operator()(const gr::PointOnLayer& point) const {
        return (std::hash<int>()(point.layerIdx) ^ std::hash<int>()(point.x) ^ std::hash<int>()(point.y));
    }
};

}  // namespace std