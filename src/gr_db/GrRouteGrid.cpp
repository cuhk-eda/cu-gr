#include "GrRouteGrid.h"
#include "db/Database.h"
#include "GrDatabase.h"
#include "db/Setting.h"

namespace gr {
void GrRouteGrid::init() {
    int numLayers = database.getLayerNum();
    routedWireMap.resize(numLayers);
    fixedMetalMap.resize(numLayers);
    histWireUsageMap.resize(numLayers);
    routedViaMap.resize(numLayers);

    GCellGrid::init();

    for (int l = 0; l < numLayers; l++) {
        auto dir = database.getLayerDir(l);
        routedWireMap[l].resize(getNumGrPoint(dir), vector<UsageT>(getNumGrEdge(l)));
        fixedMetalMap[l].resize(getNumGrPoint(dir), vector<std::pair<int, DBU>>(getNumGrEdge(l)));
        histWireUsageMap[l].resize(getNumGrPoint(dir), vector<double>(getNumGrEdge(l), 0));
        routedViaMap[l].resize(getNumGrPoint(X), vector<UsageT>(getNumGrPoint(Y)));
    }

    markFixedMetals();
}

void GrRouteGrid::clear() {
    routedWireMap.clear();
    fixedMetalMap.clear();
    histWireUsageMap.clear();
    routedViaMap.clear();
}

void GrRouteGrid::markFixed(int layerIdx, int gridline, int cp, int num_track, DBU avg_length) {
    fixedMetalMap[layerIdx][gridline][cp] = std::make_pair(num_track, avg_length);
}

void GrRouteGrid::useWire(int layerIdx, int gridline, int cp, double usage) {
    routedWireMap[layerIdx][gridline][cp] += usage;
}

void GrRouteGrid::useVia(int layerIdx, int x, int y, double usage) { routedViaMap[layerIdx][x][y] += usage; }

void GrRouteGrid::useWire(const GrBoxOnLayer& box) {
    int layerIdx = box.layerIdx;
    auto dir = database.getLayerDir(layerIdx);
    double usage = 1.0 / (box[dir].range() + 1);
    for (int gridline = box[dir].low; gridline <= box[dir].high; gridline++)
        for (int cp = box[1 - dir].low; cp < box[1 - dir].high; cp++) useWire(layerIdx, gridline, cp, usage);
}

void GrRouteGrid::useVia(const GrBoxOnLayer& box) {
    double usage = 1.0 / ((box[X].range() + 1) * (box[Y].range() + 1));
    for (int x = box[X].low; x <= box[X].high; x++)
        for (int y = box[Y].low; y <= box[Y].high; y++) useVia(box.layerIdx, x, y, usage);
}

void GrRouteGrid::useNet(const GrNet& net) {
    for (const auto& guide : net.wireRouteGuides) useWire(guide);

    const auto& viaGuides = net.viaRouteGuides;
    for (int g1 = 0; g1 < viaGuides.size(); g1++) {
        for (int g2 = g1 + 1; g2 < viaGuides.size(); g2++) {
            if (abs(viaGuides[g1].layerIdx - viaGuides[g2].layerIdx) != 1) continue;

            auto xIntvl = viaGuides[g1][X].IntersectWith(viaGuides[g2][X]);
            auto yIntvl = viaGuides[g1][Y].IntersectWith(viaGuides[g2][Y]);

            if (xIntvl.IsValid() && yIntvl.IsValid())
                useVia({min(viaGuides[g1].layerIdx, viaGuides[g2].layerIdx), xIntvl, yIntvl});
        }
    }
}

// Note: may accumulate the size of each recorder

void GrRouteGrid::removeWire(const GrBoxOnLayer& box) {
    int layerIdx = box.layerIdx;
    auto dir = database.getLayerDir(layerIdx);
    double usage = 1.0 / (box[dir].range() + 1);
    for (int gridline = box[dir].low; gridline <= box[dir].high; gridline++)
        for (int cp = box[1 - dir].low; cp < box[1 - dir].high; cp++) routedWireMap[layerIdx][gridline][cp] -= usage;
}

void GrRouteGrid::removeVia(const GrBoxOnLayer& box) {
    double usage = 1.0 / ((box[X].range() + 1) * (box[Y].range() + 1));
    for (int x = box[X].low; x <= box[X].high; x++)
        for (int y = box[Y].low; y <= box[Y].high; y++) routedViaMap[box.layerIdx][x][y] -= usage;
    // for (auto& pair : routedViaMap[box.layerIdx][x][y])
    //     if (pair.first == netIdx) pair.second = 0;
}

void GrRouteGrid::removeNet(GrNet& net) {
    for (const auto& guide : net.wireRouteGuides) removeWire(guide);

    const auto& viaGuides = net.viaRouteGuides;
    for (int g1 = 0; g1 < viaGuides.size(); g1++) {
        for (int g2 = g1 + 1; g2 < viaGuides.size(); g2++) {
            if (abs(viaGuides[g1].layerIdx - viaGuides[g2].layerIdx) != 1) continue;

            auto xIntvl = viaGuides[g1][X].IntersectWith(viaGuides[g2][X]);
            auto yIntvl = viaGuides[g1][Y].IntersectWith(viaGuides[g2][Y]);

            if (xIntvl.IsValid() && yIntvl.IsValid())
                removeVia({min(viaGuides[g1].layerIdx, viaGuides[g2].layerIdx), xIntvl, yIntvl});
        }
    }

    net.wireRouteGuides.clear();
    net.viaRouteGuides.clear();
}

double GrRouteGrid::getWireCapacity(const GrEdge& edge) const {
    if (abs(edge.u[X] - edge.v[X]) > 1 || abs(edge.u[Y] - edge.v[Y]) > 1)
        printlog("ERROR: in GrRouteGrid::getWireCapacity, edge len > 1");
    int layerIdx = edge.getLayerIdx();
    return getNumTracks(layerIdx, edge.u[database.getLayerDir(layerIdx)]) * wireCapDiscount;
}

double GrRouteGrid::getInCellArea(const GrPoint& point) const {
    int layerIdx = point.layerIdx;
    auto dir = database.getLayerDir(layerIdx);
    return getNumTracks(layerIdx, point[dir]);
}

double GrRouteGrid::getFixedUsage(const GrEdge& edge) const {  // get num of tracks blocked by fixed metal
    if (edge.getGrLen() != 1) printlog("Error");
    auto dir = database.getLayerDir(edge.getLayerIdx());
    return getFixedUsage(edge.getLayerIdx(), edge.u[dir], edge.u[1 - dir]);
}

DBU GrRouteGrid::getFixedLength(const GrEdge& edge) const {  // get avg length of the tracks blocked by fixed metal
    if (edge.getGrLen() != 1) printlog("Error");
    auto dir = database.getLayerDir(edge.getLayerIdx());
    return fixedMetalMap[edge.getLayerIdx()][edge.u[dir]][edge.u[1 - dir]].second;
}

double GrRouteGrid::getWireUsage(const GrEdge& edge) const {
    if (edge.getGrLen() != 1) printlog("Error");
    auto dir = database.getLayerDir(edge.getLayerIdx());
    return getWireUsage(edge.getLayerIdx(), edge.u[dir], edge.u[1 - dir]);
}

double GrRouteGrid::getViaUsage(const GrPoint& via) const { return getViaUsage(via.layerIdx, via[X], via[Y]); }

double GrRouteGrid::getCellResource(const GrPoint& point) const { return getCellResource(point.layerIdx, point.x, point.y); }

double GrRouteGrid::getInCellUsedArea(const GrPoint& point) const {
    // note: the area defined here = # of used tracks * avg_used_length (total used length of tracks in the gcell)
    // todo: consider boundary effect
    double used_area = 0;
    int layerIdx = point.layerIdx;
    auto dir = database.getLayerDir(layerIdx);
    int low_x = point.x - (dir == Y);
    int low_y = point.y - (dir == X);
    auto low_edge = GrEdge({layerIdx, low_x, low_y}, point);
    if (low_x >= 0 && low_y >= 0) {
        used_area += getFixedUsage(low_edge) + getWireUsage(low_edge);
    }
    int high_x = point.x + (dir == Y);
    int high_y = point.y + (dir == X);
    auto high_edge = GrEdge(point, {layerIdx, high_x, high_y});
    if (high_x < getNumGrPoint(X) && high_y < getNumGrPoint(Y)) {
        used_area += getFixedUsage(high_edge) + getWireUsage(high_edge);
    }
    return used_area / 2;
}

double GrRouteGrid::getFixedUsedArea(const GrEdge& edge) const {
    // a little different to the area of a point, this is the area of an edge, which is half of 2 gcell's area combined
    return getFixedUsage(edge) * getFixedLength(edge);
}

double GrRouteGrid::getInCellViaNum(const GrPoint& point) const {  // get the num of vias passing through the gcell
    double num = 0;
    for (int side = -1; side <= 0;
         side++) {  // a cell is used by both the via from lower layer and that from higher layer
        // side = -1, from lower layer to current layer; side = 0 from current layer to upper layer
        auto via_point = GrPoint({point.layerIdx + side, point.x, point.y});
        if (via_point.layerIdx < 0 || via_point.layerIdx >= database.getLayerNum() - 1) continue;
        num += getViaUsage(via_point);
    }
    return num;
}

// double GrRouteGrid::getUnitViaArea(const GrPoint& point, int side) const {
//     // side = -1, from lower layer to current layer; side = 0 from current layer to upper layer
//     auto& viaType = database.getCutLayer(point.layerIdx).defaultViaType();
//     auto viaBox = (side == 1 ? viaType.top : viaType.bot);
//
//     auto dir = database.getLayerDir(point.layerIdx);
//     int nBlockedTrack = viaBox[dir].range() * grDatabase.getNumTracks(point.layerIdx, point[dir]) /
//     grDatabase.getCoorIntvl(point, dir).range() + 1; return viaBox[1 - dir].range() * nBlockedTrack;
// }

double GrRouteGrid::getFixedUsage(int layerIdx,
                                  int gridline,
                                  int cp) const {  // get num of tracks blocked by fixed metal
    return fixedMetalMap[layerIdx][gridline][cp].first;
}

double GrRouteGrid::getWireUsage(int layerIdx, int gridline, int cp) const {
    return routedWireMap[layerIdx][gridline][cp];
}

double GrRouteGrid::getViaUsage(int layerIdx, int x, int y) const { return routedViaMap[layerIdx][x][y]; }

double GrRouteGrid::getCellResource(int layerIdx, int x, int y) const {
    auto layerDir = database.getLayerDir(layerIdx);
    double totalRsrc = grDatabase.getNumTracks(layerIdx, layerDir == X ? x : y);
    double cellUsage = 0;
    int low_x = x - (layerDir == Y);
    int low_y = y - (layerDir == X);
    if (low_x >= 0 && low_y >= 0) {
        auto low_edge = gr::GrEdge({layerIdx, low_x, low_y}, {layerIdx, x, y});
        cellUsage += grDatabase.getFixedUsage(low_edge) + grDatabase.getWireUsage(low_edge);
    }
    int high_x = x + (layerDir == Y);
    int high_y = y + (layerDir == X);
    if (high_x < grDatabase.getNumGrPoint(X) && high_y < grDatabase.getNumGrPoint(Y)) {
        auto high_edge = gr::GrEdge({layerIdx, x, y}, {layerIdx, high_x, high_y});
        cellUsage += grDatabase.getFixedUsage(high_edge) + grDatabase.getWireUsage(high_edge);
    }
    cellUsage /= 2;
    cellUsage += sqrt(grDatabase.getInCellViaNum({layerIdx, x, y})) * db::setting.unitSqrtViaUsage;
    return totalRsrc - cellUsage;
}

void GrRouteGrid::print() const { GCellGrid::print(); }

void GrRouteGrid::printAllUsageAndVio() const {
    const int width = 10;
    auto wlVia = printAllUsage();
    double numShort = printAllVio();
    log() << "--- Estimated Scores ---" << std::endl;
    vector<std::string> items = {"wirelength", "# vias", "short"};
    vector<double> metrics = {wlVia.first, wlVia.second, numShort};
    vector<double> weights = {db::setting.weightWirelength, db::setting.weightViaNum, db::setting.weightShortArea};
    double totalScore = 0;
    for (int i = 0; i < items.size(); ++i) {
        totalScore += metrics[i] * weights[i];
    }
    log() << std::setw(width) << "item"
          << " | " << std::setw(width + 2) << "metric"
          << " | " << std::setw(width) << "weight"
          << " | " << std::setw(width + 2) << "score"
          << " | " << std::setw(width) << "\%" << std::endl;
    for (int i = 0; i < items.size(); ++i) {
        double score = metrics[i] * weights[i];
        log() << std::setw(width) << items[i] << " | " << std::setw(width + 2) << metrics[i] << " | "
              << std::setw(width) << weights[i] << " | " << std::setw(width + 2) << score << " | " << std::setw(width)
              << score / totalScore << std::endl;
    }
    log() << "total score = " << totalScore << std::endl;
}

double GrRouteGrid::getAllWireUsage(const vector<double>& buckets,
                                    vector<int>& wireUsageGrid,
                                    vector<DBU>& wireUsageLength) const {
    double wirelength = 0;
    wireUsageGrid.assign(buckets.size(), 0);
    wireUsageLength.assign(buckets.size(), 0);
    for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
        Dimension dir = database.getLayerDir(layerIdx);

        for (int gridline = 0; gridline < getNumGrPoint(dir); gridline++) {
            for (int cp = 0; cp < getNumGrEdge(layerIdx); cp++) {
                double numWire = getWireUsage(layerIdx, gridline, cp);
                double usage = (numWire + getFixedUsage({layerIdx, gridline, cp})) / getNumTracks(layerIdx, gridline);
                DBU dist = (getCoor(cp + 2, 1 - dir) - getCoor(cp, 1 - dir)) / 2;
                int bucketIdx = buckets.size() - 1;
                while (buckets[bucketIdx] >= usage) --bucketIdx;
                bucketIdx = max(bucketIdx, 0);
                wireUsageGrid[bucketIdx]++;
                wireUsageLength[bucketIdx] += dist;
                wirelength += dist * numWire;
            }
        }
    }

    return wirelength;
}

double GrRouteGrid::getWirelength() const {
    double wirelength = 0;
    for (auto& net : grDatabase.nets) wirelength += net.getWirelength();
    return wirelength;
}

void GrRouteGrid::getAllInCellUsage(const vector<double>& buckets, vector<int>& inCellUsage) const {
    inCellUsage.assign(buckets.size(), 0);
    for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
        for (int x = 0; x < getNumGrPoint(X); x++) {
            for (int y = 0; y < getNumGrPoint(Y); y++) {
                double usage = getInCellUsedArea({layerIdx, x, y}) / getInCellArea({layerIdx, x, y});
                int bucketIdx = buckets.size() - 1;
                while (buckets[bucketIdx] >= usage) --bucketIdx;
                bucketIdx = max(bucketIdx, 0);
                inCellUsage[bucketIdx]++;
            }
        }
    }
}

double GrRouteGrid::getTotViaNum() const {
    double viaNum = 0;
    for (int layerIdx = 0; (layerIdx + 1) < database.getLayerNum(); ++layerIdx) {
        for (int x = 0; x < getNumGrPoint(X); x++) {
            for (int y = 0; y < getNumGrPoint(Y); y++) {
                viaNum += getViaUsage(layerIdx, x, y);
            }
        }
    }
    return viaNum;
}

std::pair<double, double> GrRouteGrid::printAllUsage() const {
    const int width = 10;
    vector<double> buckets = {
        -1, 0, 0.3, 0.6, 0.8, 0.9, 1, 1.1, 1.3, 1.5, 2, 3};  // the i-th bucket: buckets[i] <= x < buckets[i+1]

    auto getRangeStr = [](const vector<double>& buckets, int i) {
        std::string range;
        if (i == 0) {
            range = "      " + std::to_string_with_precision(0.0, 2) + " ";
        } else if ((i + 1) < buckets.size()) {
            range = "(" + std::to_string_with_precision(buckets[i], 2) + "~" +
                    std::to_string_with_precision(buckets[i + 1], 2) + "]";
        } else {
            range = "(" + std::to_string_with_precision(buckets[i], 2) + "~inf" + ")";
        }
        return range;
    };

    // Wire
    vector<int> routedWireUsageGrid;
    vector<DBU> routedWireUsageLength;
    double wireLength = getAllWireUsage(buckets, routedWireUsageGrid, routedWireUsageLength);
    log() << "--- Wire Usage ---" << std::endl;
    log() << std::setw(width) << "usage"
          << " | " << std::setw(width) << "   grid   "
          << " | " << std::setw(width) << "  length  " << std::endl;
    for (int i = 0; i < buckets.size(); ++i) {
        if (routedWireUsageGrid[i] == 0 && routedWireUsageLength[i] == 0) continue;

        log() << std::setw(width) << getRangeStr(buckets, i) << " | " << std::setw(width) << routedWireUsageGrid[i]
              << " | " << std::setw(width) << routedWireUsageLength[i] / double(database.getLayer(1).pitch)
              << std::endl;
    }
    wireLength /= double(database.getLayer(1).pitch);

    // in-Cell
    vector<int> routedViaUsage;
    getAllInCellUsage(buckets, routedViaUsage);
    log() << "--- in-Cell Usage ---" << std::endl;
    log() << std::setw(width) << "usage"
          << " | " << std::setw(width) << "routed" << std::endl;
    for (int i = 0; i < buckets.size(); ++i) {
        if (routedViaUsage[i] == 0) continue;

        log() << std::setw(width) << getRangeStr(buckets, i) << " | " << std::setw(width) << routedViaUsage[i]
              << std::endl;
    }

    double viaNum = getTotViaNum();
    return {wireLength, viaNum};
}

double GrRouteGrid::printAllVio() const {
    const int width = 10;
    auto sumVec = [](const vector<int>& vec) {
        int sum = 0;
        for (int val : vec) {
            sum += val;
        }
        return sum;
    };

    // Wire violations
    vector<double> shortLen(database.getLayerNum(), 0.0);
    for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
        Dimension dir = database.getLayerDir(layerIdx);

        for (int gridline = 0; gridline < getNumGrPoint(dir); gridline++) {
            for (int cp = 0; cp < getNumGrEdge(layerIdx); cp++) {
                double numWire = getWireUsage(layerIdx, gridline, cp) + getFixedUsage(layerIdx, gridline, cp);
                DBU dist = (getCoor(cp + 2, 1 - dir) - getCoor(cp, 1 - dir)) / 2;

                double overflow = max(0.0, numWire - getNumTracks(layerIdx, gridline));
                shortLen[layerIdx] += overflow * dist;
            }
        }
    }

    log() << "--- Wire-Wire Short Vios ---" << std::endl;
    log() << std::setw(width) << "usage"
          << " | " << std::setw(width * 2 + 3) << "      short area     " << std::endl;
    log() << std::setw(width) << "layer"
          << " | " << std::setw(width) << "wire-wire" << std::endl;
    double routedShortArea = 0;
    for (int i = 0; i < database.getLayerNum(); ++i) {
        if (shortLen[i] == 0) continue;
        const auto& layer = database.getLayer(i);
        double routedArea = double(shortLen[i]) * layer.width / database.getLayer(1).pitch / database.getLayer(1).pitch;
        log() << std::setw(width) << database.getLayer(i).name << " | " << std::setw(width) << routedArea << std::endl;
        routedShortArea += routedArea;
    }
    log() << std::setw(width) << "SumW"
          << " | " << std::setw(width) << routedShortArea << std::endl;

    // Via violations
    vector<double> shortNum(database.getLayerNum() - 1, 0.0);
    for (int layerIdx = 0; layerIdx < database.getLayerNum() - 1; ++layerIdx) {
        for (int x = 0; x < getNumGrPoint(X); x++) {
            for (int y = 0; y < getNumGrPoint(Y); y++) {
                double overflow = getNumVio(GrPoint({layerIdx, x, y}), 0);
                shortNum[layerIdx] += overflow;
            }
        }
    }

    log() << "--- Via-Via Short Vios ---" << std::endl;
    log() << std::setw(width) << "usage"
          << " | " << std::setw(width * 2 + 3) << "      #short     " << std::endl;
    log() << std::setw(width) << "layer"
          << " | " << std::setw(width) << "via-via" << std::endl;
    double routedShortViaNum = 0;
    for (int i = 0; i < database.getLayerNum() - 1; ++i) {
        if (shortNum[i] == 0) continue;
        log() << std::setw(width) << database.getCutLayer(i).name << " | " << std::setw(width) << shortNum[i]
              << std::endl;
        routedShortViaNum += shortNum[i];
    }
    log() << std::setw(width) << "SumW"
          << " | " << std::setw(width) << routedShortViaNum << std::endl;
    routedShortArea += routedShortViaNum;
    return routedShortArea;
}

void GrRouteGrid::markFixedMetals() {
    for (int l = 0; l < database.getLayerNum(); l++) {
        std::unordered_map<std::pair<int, int>,
                           vector<std::pair<utils::IntervalT<int>, DBU>>,
                           boost::hash<std::pair<int, int>>>
            markingBuffer;  // (gridline, cp) -> (interval,netIdx)

        Dimension dir = database.getLayerDir(l);
        const RTree& tree = database.getFixedMetals(l);
        auto bit = tree.qbegin(bgi::satisfies([](auto const&) { return true; })), eit = tree.qend();
        for (auto iter = bit; iter != eit; iter++) {
            const auto& pair = *iter;
            int netIdx = pair.second;

            db::BoxOnLayer box(l,
                               bg::get<bg::min_corner, 0>(pair.first),
                               bg::get<bg::min_corner, 1>(pair.first),
                               bg::get<bg::max_corner, 0>(pair.first),
                               bg::get<bg::max_corner, 1>(pair.first));

            // compute the forbid region of a fixed metal
            db::AggrParaRunSpace aggr = db::AggrParaRunSpace::DEFAULT;
            if (database.getLayer(0).parallelLength.size() <= 1) {
                // hack for ISPD'18 test cases
                aggr = db::AggrParaRunSpace::LARGER_WIDTH;
                if (min(box.width(), box.height()) == database.getLayer(box.layerIdx).width &&
                    database.getOvlpFixedMetals(box, -2).size() == 1) {
                    aggr = db::AggrParaRunSpace::DEFAULT;
                }
            } else {
                // hack for ISPD'19 test cases
                aggr = db::AggrParaRunSpace::LARGER_LENGTH;
            }
            auto forbidRegion = database.getMetalRectForbidRegion(box, aggr);
            auto gridBox = database.rangeSearch(forbidRegion,
                                                aggr == db::AggrParaRunSpace::LARGER_WIDTH);  // TODO: change to false
            if (!database.isValid(gridBox)) continue;
            box = database.getLoc(gridBox);

            auto grBox = rangeSearchGCell(box);
            auto trackIntvl = database.rangeSearchTrack(l, box[dir]);
            if (!trackIntvl.IsValid()) continue;

            // mark wire usage
            int jMin = max(grBox[1 - dir].low - 1, 0);
            int jMax = min(grBox[1 - dir].high, getNumGrPoint(1 - dir) - 2);
            for (int i = grBox[dir].low; i <= grBox[dir].high; i++) {
                for (int j = jMin; j <= jMax; j++) {
                    utils::IntervalT<DBU> gcellIntvl1 = {getCoor(j, 1 - dir), getCoor(j + 1, 1 - dir)};
                    utils::IntervalT<DBU> gcellIntvl2 = {getCoor(j + 1, 1 - dir), getCoor(j + 2, 1 - dir)};
                    utils::IntervalT<DBU> edgeIntvl = {gcellIntvl1.center(), gcellIntvl2.center()};

                    auto blocked_length = box[1 - dir].IntersectWith(edgeIntvl).range();
                    if (blocked_length > 0) {
                        auto gcellTrackIntvl = getTrackIntvl(l, i);
                        auto blockedIntvl = gcellTrackIntvl.IntersectWith(trackIntvl);
                        if (blockedIntvl.IsValid())
                            markingBuffer[std::make_pair(i, j)].emplace_back(
                                std::make_pair(blockedIntvl, blocked_length));
                    }
                }
            }
        }

        for (auto& buf : markingBuffer) {
            auto gcellTrackIntvl = getTrackIntvl(l, buf.first.first);
            vector<DBU> trackBlocked(gcellTrackIntvl.range() + 1, 0);  // blocked track length
            for (auto& pair : buf.second) {
                for (int t = pair.first.low; t <= pair.first.high; t++)
                    trackBlocked[t - gcellTrackIntvl.low] += pair.second;
            }
            int num_blocked = 0;
            DBU avg_blocked_len = 0;

            for (auto& len : trackBlocked) {
                if (len > 0) {
                    num_blocked++;
                    avg_blocked_len += len;
                }
            }
            avg_blocked_len /= num_blocked;
            markFixed(l, buf.first.first, buf.first.second, num_blocked, avg_blocked_len);
        }
    }
}

db::CostT GrRouteGrid::getViaCost(const GrPoint& via) const {
    return database.getUnitViaCost() * (unitViaMultiplier + getViaShortCost(via));
}

db::CostT GrRouteGrid::getStackViaCost(const GrPoint& via, int height) const {
    db::CostT cost = 0;
    for (int i = 0; i < height; i++) {
        cost += getViaCost(gr::GrPoint(via.layerIdx + i, via.x, via.y));
    }
    return cost;
};

db::CostT GrRouteGrid::getWireDistCost(const GrEdge& edge) const {
    Dimension dir = database.getLayerDir(edge.getLayerIdx());
    return getDist(edge.lowerGrPoint(), edge.upperGrPoint(), 1 - dir);
}

// logistic cost, ref: nctugr (8)
// " With a growing number of rip-up and rerouting iterations,
// the value of slope also grows to free the edge usage constraint. "

db::CostT GrRouteGrid::getWireShortCost(const GrEdge& edge) const {
    // Note: didn't consider occurrence penalty
    int layerIdx = edge.getLayerIdx();
    auto dir = database.getLayerDir(layerIdx);

    db::CostT cost = 0;

    int gridline = edge.u[dir];
    for (int i = edge.u[1 - dir]; i < edge.v[1 - dir]; i++) {
        GrEdge tempEdge(layerIdx, gridline, i);

        auto fixed_used = getFixedUsage(tempEdge);

        auto wire_used = getWireUsage(tempEdge);
        DBU expected_of_len = fixed_used * getFixedLength(tempEdge) + wire_used * getWireDistCost(tempEdge);
        expected_of_len /= grDatabase.getNumTracks(layerIdx, edge.u[dir]);
        auto demand = fixed_used + wire_used + 1;
        demand += sqrt((getInCellViaNum(tempEdge.u) + getInCellViaNum(tempEdge.v)) / 2) * db::setting.unitSqrtViaUsage;
        auto capacity = getWireCapacity(tempEdge);

        cost += expected_of_len / (1.0 + exp(-logisticSlope * (demand - capacity)));
    }

    return cost * database.getUnitShortCost(layerIdx);
}

db::CostT GrRouteGrid::getViaShortCost(const GrPoint& via) const {
    double cost = 0;
    for (int side = 0; side <= 1; side++) {
        auto side_point = GrPoint({via.layerIdx + side, via.x, via.y});
        double incell_used = getInCellUsedArea(side_point);
        double incell_area = getInCellArea(side_point);
        cost += 1.0 / (1.0 + exp(-logisticSlope * (incell_used - incell_area)));
    }
    return cost;
    // return cost;
}

db::CostT GrRouteGrid::getWireCost(const GrEdge& edge) const {
    if (edge.getLayerIdx() == 0) return LARGE_NUM;
    return getWireDistCost(edge) + getWireShortCost(edge);
    // return db::setting.wirelenCostWeight*getWireDistCost(edge) + getWireShortCost(edge);
}

bool GrRouteGrid::hasVio(const GrNet& net, bool hasCommit) const { return getNumVio(net, hasCommit) > 0; }

bool GrRouteGrid::hasVio(const GrEdge& edge, bool hasCommit) const { return getNumVio(edge, !hasCommit) > 0; }

bool GrRouteGrid::hasVio(const GrPoint& via, bool hasCommit) const { return getNumVio(via, !hasCommit) > 0; }

double GrRouteGrid::getNumVio(const GrNet& net, bool hasCommit) const {
    double numVio = 0;

    const auto& guides = net.wireRouteGuides;
    for (const auto& guide : guides) {
        auto dir = database.getLayerDir(guide.layerIdx);
        double usage = 1.0 / (guide[dir].range() + 1);
        for (int gridline = guide[dir].low; gridline <= guide[dir].high; gridline++) {
            GrEdge tempEdge = (dir == X) ? GrEdge(guide.layerIdx, gridline, guide[Y].low, guide[Y].high)
                                         : GrEdge(guide.layerIdx, gridline, guide[X].low, guide[X].high);
            numVio += getNumVio(tempEdge, hasCommit ? 0 : usage);
        }
    }

    const auto& viaGuides = net.viaRouteGuides;
    for (int g1 = 0; g1 < viaGuides.size(); g1++) {
        for (int g2 = g1 + 1; g2 < viaGuides.size(); g2++) {
            if (abs(viaGuides[g1].layerIdx - viaGuides[g2].layerIdx) != 1) continue;

            auto xIntvl = viaGuides[g1][X].IntersectWith(viaGuides[g2][X]);
            auto yIntvl = viaGuides[g1][Y].IntersectWith(viaGuides[g2][Y]);

            if (xIntvl.IsValid() && yIntvl.IsValid()) {
                double usage = 1.0 / ((xIntvl.range() + 1) * (yIntvl.range() + 1));

                for (int x = xIntvl.low; x <= xIntvl.high; x++)
                    for (int y = yIntvl.low; y <= yIntvl.high; y++)
                        numVio += getNumVio(GrPoint(min(viaGuides[g1].layerIdx, viaGuides[g2].layerIdx), x, y),
                                            hasCommit ? 0 : usage);
            }
        }
    }

    return numVio;
}

double GrRouteGrid::getNumVio(const GrEdge& edge, double selfUsage) const {
    int layerIdx = edge.getLayerIdx();
    auto dir = database.getLayerDir(layerIdx);
    int gridline = edge.u[dir];

    double numVio = 0;

    for (int i = edge.u[1 - dir]; i < edge.v[1 - dir]; i++) {
        GrEdge tempEdge(layerIdx, gridline, i);
        numVio += max(
            0.0,
            getWireUsage(tempEdge) + selfUsage + getFixedUsage(tempEdge) +
                sqrt((getInCellViaNum(tempEdge.u) + getInCellViaNum(tempEdge.v)) / 2) * db::setting.unitSqrtViaUsage -
                getWireCapacity(tempEdge));
    }
    return numVio;
}

double GrRouteGrid::getNumVio(const GrPoint& via, double selfUsage) const {
    double via_usage = getViaUsage(via) + selfUsage;
    double numVio = 0;
    for (int side = 0; side <= 1; side++) {
        auto side_point = GrPoint({via.layerIdx + side, via.x, via.y});
        auto incell_area = getInCellArea(side_point);
        auto incell_used_area = getInCellUsedArea(side_point);
        if (incell_area >= incell_used_area) {  // have remaining area

        } else {  // have no remaining area
            numVio += via_usage;
        }
    }
    // if (numVio != 0)
    // log() << numVio << std::endl;
    return numVio;
}

void GrRouteGrid::setViaCapDiscount(double discount) {
    printflog("viaCapDiscount change: %.2f->%.2f\n", viaCapDiscount, discount);
    viaCapDiscount = discount;
}

void GrRouteGrid::setWireCapDiscount(double discount) {
    printflog("wireCapDiscount change: %.2f->%.2f\n", wireCapDiscount, discount);
    wireCapDiscount = discount;
}

void GrRouteGrid::setUnitViaMultiplier(double multiplier) {
    printflog("unitViaMultiplier change: %.2f->%.2f\n", unitViaMultiplier, multiplier);
    unitViaMultiplier = multiplier;
}

void GrRouteGrid::setLogisticSlope(double slope) {
    printflog("logisticSlope change: %.2f->%.2f\n", logisticSlope, slope);
    logisticSlope = slope;
}

void GrRouteGrid::addHistCost() {
    if (db::setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        printlog("Add hist cost");
    }

    for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
        auto dir = database.getLayerDir(layerIdx);
        for (int g = 0; g < getNumGrPoint(dir); ++g) {
            for (int cp = 0; cp < getNumGrEdge(layerIdx); ++cp) {
                GrEdge tempEdge(layerIdx, g, cp);
                if (hasVio(tempEdge)) useHistWire(layerIdx, g, cp, 1);
            }
        }
    }
}

void GrRouteGrid::useHistWire(int layerIdx, int gridline, int cp, double usage) {
    histWireUsageMap[layerIdx][gridline][cp] += usage;
}

void GrRouteGrid::fadeHistCost() {
    if (db::setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        printlog("Fade hist cost by", db::setting.rrrFadeCoeff, "...");
    }
    for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
        auto dir = database.getLayerDir(layerIdx);
        // wire
        for (int g = 0; g < getNumGrPoint(dir); ++g)
            for (int cp = 0; cp < getNumGrEdge(layerIdx); ++cp)
                histWireUsageMap[layerIdx][g][cp] *= db::setting.rrrFadeCoeff;
    }
}

void GrRouteGrid::statHistCost() const {
    if (db::setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        std::map<db::CostT, int> histWireUsage, histViaUsage;
        for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
            auto dir = database.getLayerDir(layerIdx);
            // wire
            for (int g = 0; g < getNumGrPoint(dir); ++g)
                for (int cp = 0; cp < getNumGrEdge(layerIdx); ++cp) ++histWireUsage[histWireUsageMap[layerIdx][g][cp]];
            // via
        }
        printlog("Hist wire usage is", histWireUsage);
        printlog("Hist via usage is", histViaUsage);
    }
}

void GrRouteGrid2D::init2DMaps(const GrRouteGrid& routeGrid) {
    int numLayers = database.getLayerNum();
    wireUsageMap2D.resize(2);
    fixedMetalMap2D.resize(2);
    capacityMap2D.resize(2);

    int xNumGrPoint = routeGrid.getNumGrPoint(X);
    int yNumGrPoint = routeGrid.getNumGrPoint(Y);
    int xNumGrEdge = yNumGrPoint - 1;
    int yNumGrEdge = xNumGrPoint - 1;
    wireUsageMap2D[X].resize(xNumGrPoint, vector<double>(xNumGrEdge, 0));
    fixedMetalMap2D[X].resize(xNumGrPoint, vector<double>(xNumGrEdge, 0));
    capacityMap2D[X].resize(xNumGrPoint, vector<double>(xNumGrEdge, 0));

    wireUsageMap2D[Y].resize(yNumGrPoint, vector<double>(yNumGrEdge, 0));
    fixedMetalMap2D[Y].resize(yNumGrPoint, vector<double>(yNumGrEdge, 0));
    capacityMap2D[Y].resize(yNumGrPoint, vector<double>(yNumGrEdge, 0));

    for (int l = 0; l < numLayers; l++) {
        auto dir = database.getLayerDir(l);
        if (dir == X) {
            for (int gridline = 0; gridline < xNumGrPoint; gridline++) {
                for (int cp = 0; cp < xNumGrEdge; cp++) {
                    GrEdge tempEdge(l, gridline, cp);
                    fixedMetalMap2D[X][gridline][cp] += routeGrid.getFixedUsage(tempEdge);
                    capacityMap2D[X][gridline][cp] += routeGrid.getWireCapacity(tempEdge);
                }
            }
        } else {
            for (int gridline = 0; gridline < yNumGrPoint; gridline++) {
                for (int cp = 0; cp < yNumGrEdge; cp++) {
                    GrEdge tempEdge(l, gridline, cp);
                    fixedMetalMap2D[Y][gridline][cp] += routeGrid.getFixedUsage(tempEdge);
                    capacityMap2D[Y][gridline][cp] += routeGrid.getWireCapacity(tempEdge);
                }
            }
        }
    }
}

void GrRouteGrid2D::useWire2D(int dir, int gridline, int cp, double usage) {
    wireUsageMap2D[dir][gridline][cp] += usage;
}
void GrRouteGrid2D::removeUsage2D(int dir, int gridline, int cp, double usage) {
    wireUsageMap2D[dir][gridline][cp] -= usage;
}
double GrRouteGrid2D::getCost2D(int dir, int gridline, int cp) const {
    double cost = 0;

    double fixed_usage = fixedMetalMap2D[dir][gridline][cp];
    double wire_usage = wireUsageMap2D[dir][gridline][cp];
    double cap = capacityMap2D[dir][gridline][cp];

    double demand = fixed_usage + wire_usage;
    cost += 1 / (1.0 + exp(-1 * grDatabase.getLogisticSlope() * (demand - cap)));
    return cost;
}

}  // namespace gr
