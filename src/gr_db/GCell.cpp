#include "GCell.h"

namespace gr {

void GCellGrid::init() {
    db::RsynService rsynService;
    rsynService.init();
    const Rsyn::Session session;
    Rsyn::PhysicalDesign physicalDesign =
        static_cast<Rsyn::PhysicalService *>(session.getService("rsyn.physical"))->getPhysicalDesign();
    const DBU libDBU = physicalDesign.getDatabaseUnits(Rsyn::LIBRARY_DBU);

    grid.resize(2);
    grid[0].push_back(database.dieRegion[X].low);
    grid[1].push_back(database.dieRegion[Y].low);

    for (const Rsyn::PhysicalGCell &rsynGCell : physicalDesign.allPhysicalGCell()) {
        int location = rsynGCell.getLocation();
        int step = rsynGCell.getStep();
        int numStep = rsynGCell.getNumTracks();
        Dimension direction = rsynGCell.getDirection() == Rsyn::PhysicalGCellDirection::VERTICAL ? X : Y;

        for (int i = 1; i < numStep; i++) grid[direction].push_back(location + step * i);
    }

    sort(grid[X].begin(), grid[X].end());
    sort(grid[Y].begin(), grid[Y].end());

    if (grid[X].back() != database.dieRegion[X].high) grid[X].push_back(database.dieRegion[X].high);
    if (grid[Y].back() != database.dieRegion[Y].high) grid[Y].push_back(database.dieRegion[Y].high);

    numTracks.resize(database.getLayerNum());
    for (int i = 0; i < database.getLayerNum(); i++) {
        Dimension dir = database.getLayerDir(i);
        numTracks[i].resize(grid[dir].size() - 1);
        for (unsigned g = 0; g < grid[dir].size() - 1; g++) {
            utils::IntervalT<DBU> coorIntvl(grid[dir][g], grid[dir][g + 1]);
            if (grid[dir][g] >= database.getLayer(i).lastTrackLoc()) {
                numTracks[i][g] = numTracks[i][g - 1];
            } else {
                auto trackIntvl = database.rangeSearchTrack(i, coorIntvl);
                bool includeBnd = database.getLayer(i).tracks[trackIntvl.high].location == coorIntvl.high;
                numTracks[i][g] = (g == 0 ? 0 : numTracks[i][g - 1]) + trackIntvl.range() + 1 - includeBnd;
            }
        }
    }
}

GrBoxOnLayer GCellGrid::rangeSearchGCell(const db::BoxOnLayer &box) const {
    return GrBoxOnLayer(box.layerIdx, rangeSearchGCell(box[X], X), rangeSearchGCell(box[Y], Y));
}

utils::IntervalT<int> GCellGrid::rangeSearchGCell(const utils::IntervalT<DBU> &intvl, Dimension dir) const {
    int lo_idx = lower_bound(grid[dir].begin(), grid[dir].end(), intvl.low) - grid[dir].begin();
    if (grid[dir][lo_idx] > intvl.low) lo_idx--;
    int hi_idx = lower_bound(grid[dir].begin(), grid[dir].end(), intvl.high) - grid[dir].begin();
    hi_idx--;

    return utils::IntervalT<int>(lo_idx, hi_idx);
}

vector<vector<GrEdge>> GCellGrid::rangeSearchGCellEdge(const db::BoxOnLayer &box) const {
    Dimension dir = database.getLayerDir(box.layerIdx);

    auto grBox = rangeSearchGCell(box);

    vector<vector<GrEdge>> edges;
    edges.resize(grBox[dir].range() + 1);

    int jMin = max(grBox[1 - dir].low, 0);
    int jMax = min(grBox[1 - dir].high, getNumGrPoint(1 - dir) - 2);
    for (int i = grBox[dir].low; i < grBox[dir].high; i++) {
        for (int j = jMin; j <= jMax; j++) {
            int lx, ly, hx, hy;
            if (dir == X) {
                lx = hx = i;
                ly = j;
                hy = j + 1;
            } else {
                ly = hy = i;
                lx = j;
                hx = j + 1;
            }
            GrPoint p1(box.layerIdx, lx, ly), p2(box.layerIdx, hx, hy);
            edges[i].emplace_back(p1, p2);
        }
    }

    return edges;
}

utils::IntervalT<int> GCellGrid::getTrackIntvl(const GrPoint &point) const {
    return getTrackIntvl(point.layerIdx, point[database.getLayerDir(point.layerIdx)]);
}

utils::IntervalT<int> GCellGrid::getTrackIntvl(int layerIdx, int idx) const {
    return {idx == 0 ? 0 : numTracks[layerIdx][idx - 1], numTracks[layerIdx][idx] - 1};
}

void GCellGrid::print() const {
    log() << "========= gcell grid info ============" << std::endl;
    printflog("#gcell=%d*%d=%d, #edge=%d/%d\n",
              getNumGrLine(X) - 1,
              getNumGrLine(Y) - 1,
              (getNumGrLine(X) - 1) * (getNumGrLine(Y) - 1),
              (getNumGrLine(X) - 1) * (getNumGrLine(Y) - 2),
              (getNumGrLine(X) - 2) * (getNumGrLine(Y) - 1));
    for (int l = 0; l < database.getLayerNum(); l++) {
        int totTrackNum = 0;
        for (unsigned i = 0; i < numTracks[l].size(); i++) totTrackNum += getNumTracks(l, i);

        printlog(database.getLayer(l).name, ": avg #track=", totTrackNum / numTracks[l].size());
    }
}
}  // namespace gr