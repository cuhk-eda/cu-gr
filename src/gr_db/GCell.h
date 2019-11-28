#pragma once

#include "db/RsynService.h"
#include "global.h"
#include "GrGeoPrimitive.h"
#include "db/Database.h"

namespace gr {
class GCellGrid {
public:
    void init();

    int getNumTracks(int layerIdx, int idx) const { return getTrackIntvl(layerIdx, idx).range() + 1; }
    utils::IntervalT<int> getTrackIntvl(const GrPoint &point) const;

    utils::IntervalT<DBU> getCoorIntvl(const GrPoint &point, Dimension dir) const {
        return getCoorIntvl(point, (int)dir);
    }
    utils::IntervalT<DBU> getCoorIntvl(const GrPoint &point, int dir) const {
        return {getCoor(point[dir], dir), getCoor(point[dir] + 1, dir)};
    }
    DBU getDist(const GrPoint &point1, const GrPoint &point2) const {
        return getDist(point1, point2, X) + getDist(point1, point2, Y);
    }
    DBU getDist(const GrPoint &point1, const GrPoint &point2, Dimension dir) const {
        return getDist(point1, point2, (int)dir);
    }
    DBU getDist(const GrPoint &point1, const GrPoint &point2, int dir) const {
        return abs(getCoorIntvl(point1, dir).center() - getCoorIntvl(point2, dir).center());
    }

    DBU getCoor(int idx, Dimension dir) const { return getCoor(idx, (int)dir); }
    DBU getCoor(int idx, int dir) const { return grid[dir][idx]; }

    int getNumGrPoint(Dimension dir) const { return getNumGrPoint((int)dir); }
    int getNumGrPoint(int dir) const { return grid[dir].size() - 1; }
    int getNumGrLine(Dimension dir) const { return getNumGrLine((int)dir); }
    int getNumGrLine(int dir) const { return grid[dir].size(); }
    int getNumGrEdge(int layerIdx) const { return getNumGrPoint(1 - database.getLayerDir(layerIdx)) - 1; }

    GrBoxOnLayer rangeSearchGCell(const db::BoxOnLayer &box) const;
    utils::IntervalT<int> rangeSearchGCell(const utils::IntervalT<DBU> &intvl, Dimension dir) const;
    vector<vector<GrEdge>> rangeSearchGCellEdge(const db::BoxOnLayer &box) const;

    void print() const;

protected:
    vector<vector<int>> numTracks;
    vector<vector<DBU>> grid;

    utils::IntervalT<DBU> getXIntvl(int idx) const { return {getX(idx), getX(idx + 1)}; }
    utils::IntervalT<DBU> getYIntvl(int idx) const { return {getY(idx), getY(idx + 1)}; }

    DBU getX(int idx) const { return getCoor(idx, X); }
    DBU getY(int idx) const { return getCoor(idx, Y); }

    utils::IntervalT<int> getTrackIntvl(int layerIdx, int idx) const;
};

}  // namespace gr