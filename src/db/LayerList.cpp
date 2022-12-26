#include <fstream>

#include "LayerList.h"

namespace db {

void LayerList::init() {
    const Rsyn::Session session;
    Rsyn::PhysicalDesign physicalDesign =
        static_cast<Rsyn::PhysicalService*>(session.getService("rsyn.physical"))->getPhysicalDesign();
    const DBU libDBU = physicalDesign.getDatabaseUnits(Rsyn::LIBRARY_DBU);

    //  Rsyn::PhysicalLayer (LEF)
    vector<Rsyn::PhysicalLayer> rsynLayers;
    vector<Rsyn::PhysicalLayer> rsynCutLayers;
    for (const Rsyn::PhysicalLayer& rsynLayer : physicalDesign.allPhysicalLayers()) {
        switch (rsynLayer.getType()) {
            case Rsyn::ROUTING:
                rsynLayers.push_back(rsynLayer);
                break;
            case Rsyn::CUT:
                rsynCutLayers.push_back(rsynLayer);
                break;
            default:
                break;
        }
    }
    if (rsynCutLayers.size() + 1 != rsynLayers.size()) {
        log() << "Error in " << __func__ << ": rsynCutLayers.size() is " << rsynCutLayers.size()
              << " , rsynLayers.size() is " << rsynLayers.size() << " , not matched... " << std::endl;
    }

    //  Rsyn::PhysicalVia (LEF)
    vector<vector<Rsyn::PhysicalVia>> rsynVias(rsynCutLayers.size());
    for (const Rsyn::PhysicalVia& rsynVia : physicalDesign.allPhysicalVias()) {
        rsynVias[rsynVia.getCutLayer().getRelativeIndex()].push_back(rsynVia);
    }

    //  Rsyn::PhysicalTracks (DEF)
    vector<Rsyn::PhysicalTracks> rsynTracks(rsynLayers.size());
    for (const Rsyn::PhysicalTracks& rsynTrack : physicalDesign.allPhysicalTracks()) {
        int idx = rsynTrack.allLayers().front().getRelativeIndex();
        if ((rsynTrack.getDirection() == Rsyn::TRACK_HORIZONTAL) ==
            !strcmp(rsynLayers[idx].getLayer()->direction(), "HORIZONTAL")) {
            assert(rsynLayers[idx].getRelativeIndex() == idx);
            rsynTracks[idx] = (rsynTrack);
        }
    }

    // init each MetalLayer
    layers.clear();
    for (unsigned i = 0; i != rsynLayers.size(); ++i) {
        layers.emplace_back(rsynLayers[i], rsynTracks[i], libDBU);
    }

    //  init MetalLayer::CrossPointSet
    initCrossPoints();

    cutLayers.clear();
    for (unsigned i = 0; i != rsynCutLayers.size(); ++i) {
        cutLayers.emplace_back(rsynCutLayers[i], rsynVias[i], layers[i].direction, layers[i + 1].direction, libDBU);
    }
    
    /* DR Lagacy Not Used
    // via area equivalent length (conservative)
    for (int i = 0, sz = rsynLayers.size(); i < sz; i++) {
        MetalLayer& layer = layers[i];

        auto dir = layer.direction;
        auto width = layer.width;

        auto getEqLen = [&](const utils::BoxT<DBU>& rect, DBU& viaLenEqLen, DBU& viaWidthEqLen) {
            DBU minViaLen = min(abs(rect[1 - dir].low), abs(rect[1 - dir].high));  // conservative
            DBU viaWidth = rect[dir].range();
            DBU viaLen = rect[1 - dir].range();
            viaWidthEqLen = viaLen * max((DBU)0, viaWidth - width) / width;
            viaLenEqLen = max((DBU)0, minViaLen - width / 2);
        };

        DBU topViaLenEqLen = INT_MAX;
        DBU botViaLenEqLen = INT_MAX;
        DBU topViaWidthEqLen = INT_MAX;
        DBU botViaWidthEqLen = INT_MAX;

        if (i != 0) getEqLen(cutLayers[i - 1].defaultViaType().top, botViaLenEqLen, botViaWidthEqLen);
        if (i != sz - 1) getEqLen(cutLayers[i].defaultViaType().bot, topViaLenEqLen, topViaWidthEqLen);

        layer.viaLenEqLen = min(topViaLenEqLen, botViaLenEqLen);
        layer.viaWidthEqLen = min(topViaWidthEqLen, botViaWidthEqLen);

        layer.minLenRaw = layer.minLenRaw - layer.viaWidthEqLen;
        layer.minLenOneVia = layer.minLenRaw - 2 * layer.viaLenEqLen;
        layer.minLenTwoVia = layer.minLenOneVia - layer.viaWidthEqLen;

        if (i == 0) {
            layer.viaOvlpDist = 0;
        } else if (i == sz - 1) {
            layer.viaOvlpDist = 0;
        } else {
            auto intvl1 = cutLayers[i - 1].defaultViaType().top[1 - dir];
            auto intvl2 = cutLayers[i].defaultViaType().bot[1 - dir];
            layer.viaOvlpDist = max(abs(intvl1.high) + abs(intvl2.low), abs(intvl1.low) + abs(intvl2.high));
            layer.viaOvlpDist = min(layer.viaOvlpDist, intvl1.range());
            layer.viaOvlpDist = min(layer.viaOvlpDist, intvl2.range());
        }

        layer.minLenOneVia = max((DBU)0, layer.minLenOneVia);
        layer.minLenTwoVia = max((DBU)0, layer.minLenTwoVia);
        layer.minLenRaw = max((DBU)0, layer.minLenRaw);
    }

    //  init CutLayer::viaAccess
    initViaForbidRegions();
    initViaConfLUT();
    */
}

bool LayerList::isValid(const GridPoint& gridPt) const {
    return gridPt.layerIdx >= 0 && gridPt.layerIdx < layers.size() && layers[gridPt.layerIdx].isValid(gridPt);
}

bool LayerList::isValid(const GridBoxOnLayer& gridBox) const {
    return gridBox.layerIdx >= 0 && gridBox.layerIdx < layers.size() && layers[gridBox.layerIdx].isValid(gridBox);
}

bool LayerList::isValid(const ViaBox& viaBox) const {
    return isValid(viaBox.lower) && isValid(viaBox.upper) &&                                  // seperately
           getUpper(viaBox.lower) == viaBox.upper && getLower(viaBox.upper) == viaBox.lower;  // consistent
}

BoxOnLayer LayerList::getMetalRectForbidRegion(const BoxOnLayer& metalRect, AggrParaRunSpace aggr) const {
    const MetalLayer& layer = layers[metalRect.layerIdx];
    DBU margin[2];  // x, y
    for (int dir = 0; dir < 2; ++dir) {
        margin[dir] = layer.getSpace(metalRect, dir, aggr);
        margin[dir] += layer.width / 2;
    }
    return {metalRect.layerIdx,
            metalRect.lx() - margin[0],
            metalRect.ly() - margin[1],
            metalRect.hx() + margin[0],
            metalRect.hy() + margin[1]};
}

vector<utils::BoxT<DBU>> LayerList::getAccurateMetalRectForbidRegions(const BoxOnLayer& metalRect) const {
    const MetalLayer& layer = layers[metalRect.layerIdx];
    vector<utils::BoxT<DBU>> results;
    for (int dir = 0; dir < 2; ++dir) {
        const DBU range = metalRect[1 - dir].range();
        if (range < layer.maxEolWidth) {
            utils::BoxT<DBU> region = metalRect;
            region[1 - dir].low -= layer.maxEolWithin;
            region[1 - dir].high += layer.maxEolWithin;
            region[dir].low -= layer.maxEolSpace;
            region[dir].high += layer.maxEolSpace;
            results.push_back(region);
        } else {
            DBU space = layer.getParaRunSpace(metalRect);
            utils::BoxT<DBU> region = metalRect;
            region[dir].low -= space;
            region[dir].high += space;
            results.push_back(region);
        }
    }
    return results;
}

void LayerList::expandBox(BoxOnLayer& box, int numPitchToExtend) const {
    DBU margin = layers[box.layerIdx].pitch * numPitchToExtend;
    box.lx() -= margin;
    box.ly() -= margin;
    box.hx() += margin;
    box.hy() += margin;
}

void LayerList::expandBox(BoxOnLayer& box, int numPitchToExtend, int dir) const {
    DBU margin = layers[box.layerIdx].pitch * numPitchToExtend;
    box[dir].low -= margin;
    box[dir].high += margin;
}

utils::IntervalT<int> LayerList::getSurroundingTrack(int layerIdx, DBU loc) const {
    assert(layerIdx >= 0 && layerIdx < layers.size());
    return layers[layerIdx].getSurroundingTrack(loc);
}

utils::IntervalT<int> LayerList::getSurroundingCrossPoint(int layerIdx, DBU loc) const {
    if (layerIdx == 0) {
        return layers[1].getSurroundingTrack(loc);
    } else if (layerIdx == layers.size() - 1) {
        return layers[layerIdx - 1].getSurroundingTrack(loc);
    } else {
        const utils::IntervalT<int>& upperTrack = layers[layerIdx + 1].getSurroundingTrack(loc);
        const utils::IntervalT<int>& lowerTrack = layers[layerIdx - 1].getSurroundingTrack(loc);
        const utils::IntervalT<int>& fromUpperTrack = layers[layerIdx + 1].getLowerCrossPointRange(upperTrack);
        const utils::IntervalT<int>& fromLowerTrack = layers[layerIdx - 1].getUpperCrossPointRange(lowerTrack);
        return fromUpperTrack.IntersectWith(fromLowerTrack);
    }
}

GridBoxOnLayer LayerList::getSurroundingGrid(int layerIdx, utils::PointT<DBU> loc) const {
    auto dir = layers[layerIdx].direction;
    return {layerIdx, getSurroundingTrack(layerIdx, loc[dir]), getSurroundingCrossPoint(layerIdx, loc[1 - dir])};
}

utils::IntervalT<int> LayerList::rangeSearchTrack(int layerIdx,
                                                  const utils::IntervalT<DBU>& locRange,
                                                  bool includeBound) const {
    assert(layerIdx >= 0 && layerIdx < layers.size());
    return layers[layerIdx].rangeSearchTrack(locRange, includeBound);
}

utils::IntervalT<int> LayerList::rangeSearchCrossPoint(int layerIdx,
                                                       const utils::IntervalT<DBU>& locRange,
                                                       bool includeBound) const {
    assert(layerIdx >= 0 && layerIdx < layers.size());
    if (layerIdx == 0) {
        return layers[1].rangeSearchTrack(locRange, includeBound);
    } else if (layerIdx == layers.size() - 1) {
        return layers[layerIdx - 1].rangeSearchTrack(locRange, includeBound);
    } else {
        const utils::IntervalT<int>& upperTrack = layers[layerIdx + 1].rangeSearchTrack(locRange, includeBound);
        const utils::IntervalT<int>& lowerTrack = layers[layerIdx - 1].rangeSearchTrack(locRange, includeBound);
        bool upperValid = layers[layerIdx + 1].isTrackRangeValid(upperTrack);
        bool lowerValid = layers[layerIdx - 1].isTrackRangeValid(lowerTrack);
        if (upperValid && lowerValid) {
            // the most typical case
            const utils::IntervalT<int>& fromUpperTrack = layers[layerIdx + 1].getLowerCrossPointRange(upperTrack);
            const utils::IntervalT<int>& fromLowerTrack = layers[layerIdx - 1].getUpperCrossPointRange(lowerTrack);
            return fromUpperTrack.UnionWith(fromLowerTrack);
        } else if (upperValid || (!lowerValid && layers[layerIdx + 1].isTrackRangeWeaklyValid(upperTrack))) {
            return layers[layerIdx + 1].getLowerCrossPointRange(upperTrack);
        } else if (layers[layerIdx - 1].isTrackRangeWeaklyValid(lowerTrack)) {
            return layers[layerIdx - 1].getUpperCrossPointRange(lowerTrack);
        } else {
            return {0, -1};  // a little bit safer than default {inf, -inf}
        }
    }
}

GridBoxOnLayer LayerList::rangeSearch(const BoxOnLayer& box, bool includeBound) const {
    auto dir = layers[box.layerIdx].direction;
    return {box.layerIdx,
            rangeSearchTrack(box.layerIdx, box[dir], includeBound),
            rangeSearchCrossPoint(box.layerIdx, box[1 - dir], includeBound)};
}

utils::PointT<DBU> LayerList::getLoc(const GridPoint& gridPt) const {
    assert(isValid(gridPt));
    return layers[gridPt.layerIdx].getLoc(gridPt);
}

BoxOnLayer LayerList::getLoc(const GridBoxOnLayer& gridBox) const {
    assert(isValid(gridBox));
    return layers[gridBox.layerIdx].getLoc(gridBox);
}

std::pair<utils::PointT<DBU>, utils::PointT<DBU>> LayerList::getLoc(const GridEdge& edge) const {
    assert(edge.isTrackSegment() || edge.isWrongWaySegment());
    return layers[edge.u.layerIdx].getLoc(edge);
}

GridPoint LayerList::getUpper(const GridPoint& cur) const {
    assert(isValid(cur) && cur.layerIdx < (int)layers.size() - 1);
    return layers[cur.layerIdx].getUpper(cur);
}

GridPoint LayerList::getLower(const GridPoint& cur) const {
    assert(isValid(cur) && cur.layerIdx > 0);
    return layers[cur.layerIdx].getLower(cur);
}

GridBoxOnLayer LayerList::getUpper(const GridBoxOnLayer& cur) const {
    assert(isValid(cur) && cur.layerIdx < (int)layers.size() - 1);
    return GridBoxOnLayer(
        // layer
        cur.layerIdx + 1,
        // track
        layers[cur.layerIdx + 1].rangeSearchTrack(
            {layers[cur.layerIdx].crossPoints[cur.crossPointRange.low].location,
             layers[cur.layerIdx].crossPoints[cur.crossPointRange.high].location}),
        // cross point
        layers[cur.layerIdx].getUpperCrossPointRange(cur.trackRange));
}

GridBoxOnLayer LayerList::getLower(const GridBoxOnLayer& cur) const {
    assert(isValid(cur) && cur.layerIdx > 0);
    return GridBoxOnLayer(
        // layer
        cur.layerIdx - 1,
        // track
        layers[cur.layerIdx - 1].rangeSearchTrack(
            {layers[cur.layerIdx].crossPoints[cur.crossPointRange.low].location,
             layers[cur.layerIdx].crossPoints[cur.crossPointRange.high].location}),
        // cross point
        layers[cur.layerIdx].getLowerCrossPointRange(cur.trackRange));
}

ViaBox LayerList::getViaBoxBetween(const BoxOnLayer& lower, const BoxOnLayer& upper) {
    assert((lower.layerIdx + 1) == upper.layerIdx);
    auto box2d = lower.IntersectWith(upper);
    auto lowerGridBoxTmp = rangeSearch({lower.layerIdx, box2d});
    if (!isValid(lowerGridBoxTmp)) return ViaBox();  // invalid
    auto upperGridBox = getUpper(lowerGridBoxTmp);
    if (!isValid(upperGridBox)) return ViaBox();  // invalid
    auto lowerGridBox = getLower(upperGridBox);
    if (!isValid(lowerGridBox)) return ViaBox();  // invalid
    return ViaBox(lowerGridBox, upperGridBox);
}

bool LayerList::isConnected(const GridBoxOnLayer& lhs, const GridBoxOnLayer& rhs) {
    if (!isValid(lhs) || !isValid(rhs)) {
        return false;
    } else if ((lhs.layerIdx + 1) == rhs.layerIdx) {
        return isValid(getViaBoxBetween(lhs, rhs));
    } else if (lhs.layerIdx == (rhs.layerIdx + 1)) {
        return isValid(getViaBoxBetween(rhs, lhs));
    } else if (lhs.layerIdx == rhs.layerIdx) {
        return lhs.trackRange.HasIntersectWith(rhs.trackRange) &&
               lhs.crossPointRange.HasIntersectWith(rhs.crossPointRange);
    } else {
        return false;
    }
}

bool LayerList::isAdjacent(const GridBoxOnLayer& lhs, const GridBoxOnLayer& rhs) {
    if (!isValid(lhs) || !isValid(rhs)) {
        return false;
    } else if (lhs.layerIdx == rhs.layerIdx) {
        return (abs(lhs.trackRange.low - rhs.trackRange.high) == 1 ||
                abs(rhs.trackRange.low - lhs.trackRange.high) == 1) &&
               lhs.crossPointRange.HasIntersectWith(rhs.crossPointRange);
    } else {
        return false;
    }
}

void LayerList::initCrossPoints() {
    for (unsigned i = 0; i != layers.size(); ++i) {
        vector<CrossPoint>& crossPoints = layers[i].crossPoints;
        vector<Track> emptyTrackSet;
        vector<Track>& lowerTrackSet = (i > 0) ? layers[i - 1].tracks : emptyTrackSet;
        vector<Track>& upperTrackSet = (i < (layers.size() - 1)) ? layers[i + 1].tracks : emptyTrackSet;

        // merge cross points to lower and upper layers
        int iLo = 0, iUp = 0;  // track indexes
        DBU lastBoth = 0;
        while (iLo < lowerTrackSet.size() || iUp < upperTrackSet.size()) {
            if (iUp >= upperTrackSet.size()) {
                crossPoints.emplace_back(lowerTrackSet[iLo].location, iLo, -1);
                lowerTrackSet[iLo].upperCPIdx = crossPoints.size() - 1;
                ++iLo;
            } else if (iLo >= lowerTrackSet.size()) {
                crossPoints.emplace_back(upperTrackSet[iUp].location, -1, iUp);
                upperTrackSet[iUp].lowerCPIdx = crossPoints.size() - 1;
                ++iUp;
            }  // boundaries should be checked first
            else if (lowerTrackSet[iLo].location < upperTrackSet[iUp].location) {
                crossPoints.emplace_back(lowerTrackSet[iLo].location, iLo, -1);
                lowerTrackSet[iLo].upperCPIdx = crossPoints.size() - 1;
                ++iLo;
            } else if (lowerTrackSet[iLo].location > upperTrackSet[iUp].location) {
                crossPoints.emplace_back(upperTrackSet[iUp].location, -1, iUp);
                upperTrackSet[iUp].lowerCPIdx = crossPoints.size() - 1;
                ++iUp;
            } else {  // iLo < lowerTrackSet.size() && iUp < lowerTrackSet.size() && lowerTrackSet[iLo].location ==
                      // lowerTrackSet[iUp].location
                crossPoints.emplace_back(lowerTrackSet[iLo].location, iLo, iUp);
                lowerTrackSet[iLo].upperCPIdx = crossPoints.size() - 1;
                upperTrackSet[iUp].lowerCPIdx = crossPoints.size() - 1;
                ++iLo;
                ++iUp;
            }
        }

        layers[i].initAccCrossPointDistCost();
    }
}

void LayerList::initOppLUT(const vector<vector<vector<bool>>>& ori, vector<vector<vector<bool>>>& opp) {
    const size_t nCPs = ori.size();
    size_t xSize = 0;
    for (const vector<vector<bool>>& orig : ori) {
        xSize = orig.size();
        if (xSize) {
            break;
        }
    }

    auto travelCPs = [&](std::function<void(const unsigned, const unsigned, const unsigned, const unsigned)> handle) {
        for (unsigned i = 0; i != nCPs; ++i) {
            if (ori[i].empty()) {
                continue;
            }
            for (unsigned j = 0; j != xSize; ++j) {
                const int ySize = ((int)ori[i][0].size() - 1) / 2;
                for (unsigned k = max(0, ySize - (int)i); k < min(ySize * 2 + 1, int(nCPs + ySize - i)); ++k) {
                    if (ori[i][j][k]) {
                        handle(i, j, ySize, k);
                    }
                }
            }
        }
    };

    vector<int> ySizes(nCPs, -1);
    auto updateSize = [&](const unsigned i, const unsigned j, const unsigned ySize, const unsigned k) {
        ySizes[i + k - ySize] = max(ySizes[i + k - ySize], abs((int)ySize - (int)k));
    };
    auto fillTable = [&](const unsigned i, const unsigned j, const unsigned ySize, const unsigned k) {
        vector<bool>& tmpOpp = opp[i + k - ySize][xSize - j - 1];
        const int tmpYSize = ((int)tmpOpp.size() - 1) / 2;
        tmpOpp[tmpYSize + ySize - k] = true;
    };

    travelCPs(updateSize);
    opp.clear();
    opp.resize(nCPs);
    for (unsigned i = 0; i != nCPs; ++i) {
        if (ySizes[i] >= 0) {
            opp[i].resize(xSize, vector<bool>(ySizes[i] * 2 + 1, false));
        } else if (ori[i].empty()) {
            opp[i].resize(xSize, vector<bool>(1, false));
        }
        assert(ori[i].size() + opp[i].size());
    }
    travelCPs(fillTable);
}

void LayerList::initViaWire(const int layerIdx,
                            const utils::BoxT<DBU>& viaMetal,
                            vector<vector<vector<bool>>>& viaWireLUT) {
    const MetalLayer& layer = layers[layerIdx];
    const DBU halfWidth = ceil(layer.width / 2.0);
    const DBU viaMetalWidth = viaMetal[layer.direction].range();
    const DBU viaMetalHeight = viaMetal[1 - layer.direction].range();
    const DBU pSpace = layer.getParaRunSpace(viaMetal);
    const DBU space = max({pSpace, layer.maxEolSpace, layer.maxEolWithin});
    const size_t xSize = max(ceil((space + halfWidth + viaMetal[layer.direction].high) / (double)layer.pitch),
                             ceil((space + halfWidth - viaMetal[layer.direction].low) / (double)layer.pitch)) -
                         1;
    const utils::IntervalT<DBU> locRange(-space - halfWidth + viaMetal[1 - layer.direction].low + 1,
                                         +space + halfWidth + viaMetal[1 - layer.direction].high - 1);
    viaWireLUT.resize(layer.numCrossPoints());
    vector<bool> viaTrack(xSize + 1, false);
    for (unsigned i = 0; i != layer.numCrossPoints(); ++i) {
        const CrossPoint& cp = layer.crossPoints[i];
        utils::IntervalT<DBU> tmpLocRange(locRange);
        tmpLocRange.ShiftBy(layer.crossPoints[i].location);
        const utils::IntervalT<int>& cpRange = rangeSearchCrossPoint(layerIdx, tmpLocRange);
        const size_t ySize = max((int)i - cpRange.low, cpRange.high - (int)i);
        viaWireLUT[i].resize(xSize * 2 + 1, vector<bool>(ySize * 2 + 1, false));
        for (unsigned j = 0; j != xSize * 2 + 1; ++j) {
            DBU xDist = 0;
            if (j < xSize) {
                xDist = max(0L, int(xSize - j) * layer.pitch - halfWidth + viaMetal[layer.direction].low);
            } else if (j > xSize) {
                xDist = max(0L, int(j - xSize) * layer.pitch - halfWidth - viaMetal[layer.direction].high);
            }
            for (unsigned k = max(0, (int)ySize - (int)i); k < min(ySize * 2 + 1, layer.numCrossPoints() + ySize - i);
                 ++k) {
                const CrossPoint& tmpCP = layer.crossPoints[i + k - ySize];
                DBU yDist = 0;
                if (k < ySize) {
                    yDist = max(0L, cp.location - tmpCP.location - halfWidth + viaMetal[1 - layer.direction].low);
                } else if (k > ySize) {
                    yDist = max(0L, tmpCP.location - cp.location - halfWidth - viaMetal[1 - layer.direction].high);
                }
                if (pow(xDist, 2) + pow(yDist, 2) < pow(pSpace, 2) ||
                    layer.isEolViolation(xDist, viaMetalHeight, yDist) ||
                    layer.isEolViolation(yDist, viaMetalWidth, xDist)) {
                    viaTrack[abs((int)j - (int)xSize)] = true;
                    viaWireLUT[i][j][k] = true;
                }
            }
        }
    }
    size_t minXSize = xSize;
    for (; minXSize && !viaTrack[minXSize]; --minXSize) {
    }
    unsigned d = xSize - minXSize;
    if (!d) {
        return;
    }
    for (vector<vector<bool>>& vvb : viaWireLUT) {
        for (unsigned i = 0; i != minXSize * 2 + 1; ++i) {
            vvb[i] = vvb[i + d];
        }
        vvb.resize(minXSize * 2 + 1);
    }
}

void LayerList::initViaConfLUT() {
    for (unsigned i = 0; i != cutLayers.size(); ++i) {
        CutLayer& cutLayer = cutLayers[i];
        // Loops for init all-all via-via LUTs
        for (unsigned j = 0; j != cutLayer.allViaTypes.size(); ++j) {
            ViaType& viaType1 = cutLayer.allViaTypes[j];
            viaType1.allViaCut.resize(cutLayer.allViaTypes.size());
            viaType1.allViaMetal.resize(cutLayer.allViaTypes.size());
            viaType1.allViaMetalNum.resize(cutLayer.allViaTypes.size());
            for (unsigned k = 0; k != cutLayer.allViaTypes.size(); ++k) {
                ViaType& viaType2 = cutLayer.allViaTypes[k];
                auto& viaCut = viaType1.allViaCut[k];
                auto& viaMetal = viaType1.allViaMetal[k];
                auto& viaMetalNum = viaType1.allViaMetalNum[k];
                initSameLayerViaConfLUT(i, viaType1, viaType2, viaCut, viaMetal, viaMetalNum);
            }
        }

        //  2. init viaTopVia & viaBotVia
        if (i > 0) {
            // Loops for init all-all via-via LUTs
            for (unsigned j = 0; j != cutLayer.allViaTypes.size(); ++j) {
                ViaType& viaType1 = cutLayer.allViaTypes[j];
                viaType1.allViaBotVia.resize(cutLayers[i - 1].allViaTypes.size());
                for (unsigned k = 0; k != cutLayers[i - 1].allViaTypes.size(); ++k) {
                    ViaType& viaType2 = cutLayers[i - 1].allViaTypes[k];
                    viaType2.allViaTopVia.resize(cutLayer.allViaTypes.size());
                    auto& viaBotVia = viaType1.allViaBotVia[k];
                    auto& viaTopVia = viaType2.allViaTopVia[j];
                    initDiffLayerViaConfLUT(i, viaType1, viaType2, viaBotVia, viaTopVia);
                }
            }
        }

        //  3. init viaBotWire & viaTopWire
        layers[i].wireTopVia.resize(cutLayer.allViaTypes.size());
        layers[i + 1].wireBotVia.resize(cutLayer.allViaTypes.size());
        for (auto& viaType : cutLayer.allViaTypes) {
            initViaWire(i, viaType.bot, viaType.viaBotWire);
            initViaWire(i + 1, viaType.top, viaType.viaTopWire);
            LayerList::initOppLUT(viaType.viaBotWire, layers[i].wireTopVia[viaType.idx]);
            LayerList::initOppLUT(viaType.viaTopWire, layers[i + 1].wireBotVia[viaType.idx]);
        }
    }

    // 4. init wireRange
    for (MetalLayer& layer : layers) {
        layer.initWireRange();
    }

    // Merge LUTs
    for (unsigned i = 0; i != cutLayers.size(); ++i) {
        CutLayer& cutLayer = cutLayers[i];
        for (unsigned j = 0; j != cutLayer.allViaTypes.size(); ++j) {
            ViaType& viaType = cutLayer.allViaTypes[j];
            viaType.mergedAllViaMetal = mergeLUTs(viaType.allViaMetal);
            if (i > 0) {
                for (int k = 0; k != viaType.allViaBotVia.size(); ++k) {
                    viaType.mergedAllViaBotVia = mergeLUTsCP(viaType.allViaBotVia);
                }
            }
            if (i < cutLayers.size()) {
                for (int k = 0; k != viaType.allViaTopVia.size(); ++k) {
                    viaType.mergedAllViaTopVia = mergeLUTsCP(viaType.allViaTopVia);
                }
            }
        }
    }
    for (int i = 0; i < layers.size(); ++i) {
        MetalLayer& layer = layers[i];
        if (i > 0) {
            layer.mergedWireBotVia = mergeLUTsCP(layer.wireBotVia);
        }
        if ((i + 1) < layers.size()) {
            layer.mergedWireTopVia = mergeLUTsCP(layer.wireTopVia);
        }
    }

    // Set isWireViaMultiTrack
    for (int i = 0; i < layers.size(); ++i) {
        if (i > 0 && layers[i].wireBotVia[0][0].size() > 1 ||
            (i + 1) < layers.size() && layers[i].wireTopVia[0][0].size() > 1)
            layers[i].isWireViaMultiTrack = true;
    }

    //  writeDefConflictLUTs("debugConflictLUTa.log");
    //  exit(0);
}

void LayerList::initSameLayerViaConfLUT(const int layerIdx,
                                        ViaType& viaT1,
                                        ViaType& viaT2,
                                        vector<vector<bool>>& viaCut,
                                        vector<vector<bool>>& viaMetal,
                                        vector<vector<int>>& viaMetalNum) {
    CutLayer& cutLayer = cutLayers[layerIdx];
    MetalLayer& botLayer = layers[layerIdx];
    MetalLayer& topLayer = layers[layerIdx + 1];
    const Dimension botDim = botLayer.direction;
    const Dimension topDim = topLayer.direction;
    const DBU cutSpacing = cutLayer.spacing;

    const utils::BoxT<DBU>& botT1 = viaT1.bot;
    const utils::BoxT<DBU>& cutT1 = viaT1.cut;
    const utils::BoxT<DBU>& topT1 = viaT1.top;
    const utils::BoxT<DBU>& botT2 = viaT2.bot;
    const utils::BoxT<DBU>& cutT2 = viaT2.cut;
    const utils::BoxT<DBU>& topT2 = viaT2.top;

    const DBU botPSpace = max(botLayer.getParaRunSpace(botT1), botLayer.getParaRunSpace(botT2));
    const DBU topPSpace = max(topLayer.getParaRunSpace(topT1), topLayer.getParaRunSpace(topT2));
    const DBU botCSpace = 0;  // max(botLayer.getCornerSpace(botT1), botLayer.getCornerSpace(botT2));
    const DBU topCSpace = 0;  // max(topLayer.getCornerSpace(topT1), topLayer.getCornerSpace(topT2));
    const DBU botSpace = max({botPSpace, botCSpace, botLayer.maxEolSpace, botLayer.maxEolWithin});
    const DBU topSpace = max({topPSpace, topCSpace, topLayer.maxEolSpace, topLayer.maxEolWithin});
    const DBU botPitch = botLayer.pitch;
    const DBU topPitch = topLayer.pitch;
    // init viaCut & viaMetal
    const size_t cutXSize = max(ceil((cutSpacing + cutT1[botDim].high - cutT2[botDim].low) / (double)botPitch),
                                ceil((cutSpacing + cutT2[botDim].high - cutT1[botDim].low) / (double)botPitch)) -
                            1;
    const size_t cutYSize = max(ceil((cutSpacing + cutT1[topDim].high - cutT2[topDim].low) / (double)topPitch),
                                ceil((cutSpacing + cutT2[topDim].high - cutT1[topDim].low) / (double)topPitch)) -
                            1;
    const DBU metalXLength = max({botSpace + botT2[botDim].high - botT1[botDim].low,
                                  botSpace + botT1[botDim].high - botT2[botDim].low,
                                  topSpace + topT2[botDim].high - topT1[botDim].low,
                                  topSpace + topT1[botDim].high - topT2[botDim].low});
    const DBU metalYLength = max({botSpace + botT2[topDim].high - botT1[topDim].low,
                                  botSpace + botT1[topDim].high - botT2[topDim].low,
                                  topSpace + topT2[topDim].high - topT1[topDim].low,
                                  topSpace + topT1[topDim].high - topT2[topDim].low});
    const size_t metalXSize = max(cutXSize, (size_t)ceil((metalXLength) / (double)botPitch) - 1);
    const size_t metalYSize = max(cutYSize, (size_t)ceil((metalYLength) / (double)topPitch) - 1);
    const DBU maxLength = max(metalXLength, metalYLength);
    botLayer.confLutMargin = max(botLayer.confLutMargin, maxLength);
    topLayer.confLutMargin = max(topLayer.confLutMargin, maxLength);
    viaCut.resize(2 * cutXSize + 1, vector<bool>(2 * cutYSize + 1, false));
    viaMetal.resize(2 * metalXSize + 1, vector<bool>(2 * metalYSize + 1, false));
    viaMetalNum.resize(2 * metalXSize + 1, vector<int>(2 * metalYSize + 1, 0));
    vector<bool> viaMetalTrack(2 * metalXSize + 1, false);

    utils::PointT<DBU> delta;
    for (unsigned j = 0; j != 2 * cutXSize + 1; ++j) {
        delta[botDim] = botPitch * (static_cast<int>(j) - static_cast<int>(cutXSize));
        for (unsigned k = 0; k != 2 * cutYSize + 1; ++k) {
            delta[topDim] = topPitch * (static_cast<int>(k) - static_cast<int>(cutYSize));
            utils::BoxT<DBU> tmpT2(cutT2);
            tmpT2.ShiftBy(delta);
            if (utils::L2Dist(tmpT2, cutT1) < cutSpacing) viaCut[j][k] = true;
        }
    }

    const size_t offsetX{metalXSize - cutXSize};
    const size_t offsetY{metalYSize - cutYSize};
    for (unsigned j = 0; j != 2 * metalXSize + 1; ++j) {
        delta[botDim] = botPitch * (static_cast<int>(j) - static_cast<int>(metalXSize));
        for (unsigned k = 0; k != 2 * metalYSize + 1; ++k) {
            delta[topDim] = topPitch * (static_cast<int>(k) - static_cast<int>(metalYSize));
            utils::BoxT<DBU> tmpBotT2(botT2);
            utils::BoxT<DBU> tmpTopT2(topT2);
            tmpBotT2.ShiftBy(delta);
            tmpTopT2.ShiftBy(delta);
            viaMetalNum[j][k] +=
                static_cast<int>(j <= 2 * cutXSize + offsetX && k <= 2 * cutYSize + offsetY && j >= offsetX &&
                                 k >= offsetY && viaCut[j - offsetX][k - offsetY]) +
                static_cast<int>(
                    L2Dist(tmpBotT2, botT1) < botPSpace || L2Dist(tmpTopT2, topT1) < topPSpace ||
                    botLayer.isEolViolation(tmpBotT2, botT1) || topLayer.isEolViolation(tmpTopT2, topT1));  // ||
                    // utils::ParaRunLength(tmpBotT2, botT1) <= 0 && utils::LInfDist(tmpBotT2, botT1) < botCSpace ||
                    // utils::ParaRunLength(tmpTopT2, topT1) <= 0 && utils::LInfDist(tmpTopT2, topT1) < topCSpace);
            if (viaMetalNum[j][k]) {
                viaMetalTrack[j] = true;
                viaMetal[j][k] = true;
            }
        }
    }

    size_t minMetalXSize = 2 * metalXSize + 1;
    for (; minMetalXSize && !viaMetalTrack[minMetalXSize]; --minMetalXSize) {
    }
    if (minMetalXSize < metalXSize) viaMetal.resize(minMetalXSize + 1);
}

void LayerList::initDiffLayerViaConfLUT(const int layerIdx,
                                        ViaType& viaT1,
                                        ViaType& viaT2,
                                        vector<vector<vector<bool>>>& viaBotVia,
                                        vector<vector<vector<bool>>>& viaTopVia) {
    MetalLayer& botLayer = layers[layerIdx];
    const Dimension botDim = botLayer.direction;

    const utils::BoxT<DBU>& botT1 = viaT1.bot;
    const utils::BoxT<DBU>& topT2 = viaT2.top;

    const DBU botPSpace = max(botLayer.getParaRunSpace(botT1), botLayer.getParaRunSpace(topT2));
    const DBU botCSpace = 0;  // max(botLayer.getCornerSpace(botT1), botLayer.getCornerSpace(topT2));
    const DBU botSpace = max({botPSpace, botCSpace, botLayer.maxEolSpace, botLayer.maxEolWithin});
    const DBU botPitch = botLayer.pitch;

    const unsigned nBotCPs = botLayer.numCrossPoints();

    const DBU xLength = botSpace + max(botT1[botDim].high - topT2[botDim].low, topT2[botDim].high - botT1[botDim].low);
    const size_t xSize = ceil(xLength / (double)botPitch) - 1;
    const utils::IntervalT<DBU> botLocRange(-topT2[1 - botDim].high + botT1[1 - botDim].low - botSpace + 1,
                                            +botT1[1 - botDim].high - topT2[1 - botDim].low + botSpace - 1);
                                            
    const DBU maxLength = max<DBU>({xLength, -botLocRange.low, botLocRange.high});
    botLayer.confLutMargin = max(botLayer.confLutMargin, maxLength);
    viaBotVia.resize(nBotCPs);
    for (unsigned j = 0; j != nBotCPs; ++j) {
        utils::IntervalT<DBU> tmpLocRange(botLocRange);
        tmpLocRange.ShiftBy(botLayer.crossPoints[j].location);
        const utils::IntervalT<int>& cpRange = rangeSearchCrossPoint(layerIdx, tmpLocRange);
        const size_t ySize = max((int)j - cpRange.low, cpRange.high - (int)j);
        if (botLayer.crossPoints[j].upperTrackIdx >= 0) {
            viaBotVia[j].resize(xSize * 2 + 1, vector<bool>(ySize * 2 + 1, false));
        } else {
            viaBotVia[j].resize(xSize * 2 + 1, vector<bool>(ySize * 2 + 1, true));
        }
    }

    utils::PointT<DBU> delta;
    for (unsigned j = 0; j != nBotCPs; ++j) {
        const CrossPoint& cp = botLayer.crossPoints[j];
        if (cp.upperTrackIdx == -1) {
            continue;
        }
        const unsigned ySize = (viaBotVia[j][0].size() - 1) / 2;
        for (unsigned k = 0; k != xSize * 2 + 1; ++k) {
            delta[botDim] = botPitch * (static_cast<int>(k) - static_cast<int>(xSize));
            for (unsigned l = max(0, (int)ySize - (int)j); l < min(ySize * 2 + 1, nBotCPs + ySize - j); ++l) {
                const CrossPoint& tmpCP = botLayer.crossPoints[j + l - ySize];
                if (tmpCP.lowerTrackIdx == -1) {
                    viaBotVia[j][k][l] = true;
                    continue;
                }
                delta[1 - botDim] = tmpCP.location - cp.location;
                utils::BoxT<DBU> tmpTopT2(topT2);
                tmpTopT2.ShiftBy(delta);
                if (L2Dist(tmpTopT2, botT1) < botPSpace || botLayer.isEolViolation(tmpTopT2, botT1)) {  // ||
                    // utils::ParaRunLength(tmpTopT2, botT1) <= 0 && utils::LInfDist(tmpTopT2, botT1) < botCSpace) {
                    viaBotVia[j][k][l] = true;
                }
            }
        }
    }
    LayerList::initOppLUT(viaBotVia, viaTopVia);
}

void LayerList::initViaForbidRegions() {
    for (int i = 0; i < cutLayers.size(); ++i) {
        auto& cutLayer = cutLayers[i];
        cutLayer.botMaxForbidRegion = cutLayer.defaultViaType().bot;
        cutLayer.topMaxForbidRegion = cutLayer.defaultViaType().top;
        for (auto& viaType : cutLayer.allViaTypes) {
            viaType.botForbidRegions = getAccurateMetalRectForbidRegions({i, viaType.bot});
            for (const auto& region : viaType.botForbidRegions) {
                cutLayer.botMaxForbidRegion = cutLayer.botMaxForbidRegion.UnionWith(region);
            }
            viaType.topForbidRegions = getAccurateMetalRectForbidRegions({i + 1, viaType.top});
            for (const auto& region : viaType.topForbidRegions) {
                cutLayer.topMaxForbidRegion = cutLayer.topMaxForbidRegion.UnionWith(region);
            }
        }
    }
}

void LayerList::mergeLUT(vector<vector<bool>>& lhs, const vector<vector<bool>>& rhs) {
    const unsigned lhsXSize = lhs.size() / 2;
    const unsigned lhsYSize = lhs[0].size() / 2;
    const unsigned rhsXSize = rhs.size() / 2;
    const unsigned rhsYSize = rhs[0].size() / 2;
    const unsigned offsetX = lhsXSize - rhsXSize;
    const unsigned offsetY = lhsYSize - rhsYSize;
    for (unsigned j = 0; j != rhs.size(); ++j) {
        for (unsigned k = 0; k != rhs[0].size(); ++k) {
            if (rhs[j][k]) lhs[j + offsetX][k + offsetY] = true;
        }
    }
}

vector<vector<bool>> LayerList::mergeLUTs(const vector<vector<vector<bool>>>& LUTs) {
    int XSize = 0, YSize = 0;
    vector<vector<bool>> mergedLUT;
    for (auto& LUT : LUTs) {
        XSize = max(XSize, int(LUT.size()));
        YSize = max(YSize, int(LUT[0].size()));
    }
    mergedLUT.resize(XSize, vector<bool>(YSize, false));
    for (auto& LUT : LUTs) {
        mergeLUT(mergedLUT, LUT);
    }
    return mergedLUT;
}

vector<vector<vector<bool>>> LayerList::mergeLUTsCP(const vector<vector<vector<vector<bool>>>>& LUTs) {
    vector<int> XSizes(LUTs[0].size(), 0);
    vector<int> YSizes(LUTs[0].size(), 0);
    // crossPointIdx, trackIdx, crossPointIdx
    vector<vector<vector<bool>>> mergedLUTs(LUTs[0].size());
    for (unsigned cpIdx = 0; cpIdx != LUTs[0].size(); ++cpIdx) {
        for (unsigned typeIdx = 0; typeIdx != LUTs.size(); ++typeIdx) {
            XSizes[cpIdx] = max(XSizes[cpIdx], int(LUTs[typeIdx][cpIdx].size()));
            YSizes[cpIdx] = max(YSizes[cpIdx], int(LUTs[typeIdx][cpIdx][0].size()));
        }
        mergedLUTs[cpIdx].resize(XSizes[cpIdx], vector<bool>(YSizes[cpIdx], false));
    }

    for (unsigned cpIdx = 0; cpIdx != LUTs[0].size(); ++cpIdx) {
        for (unsigned typeIdx = 0; typeIdx != LUTs.size(); ++typeIdx) {
            mergeLUT(mergedLUTs[cpIdx], LUTs[typeIdx][cpIdx]);
        }
    }

    return mergedLUTs;
}

void LayerList::print() {
    log() << "METAL LAYERS" << std::endl;
    numGridPoints = 0;
    totalTrackLength = 0;
    for (const MetalLayer& layer : layers) {
        log() << layer << std::endl;
        numGridPoints += layer.numGridPoints();
        totalTrackLength += layer.numTracks() * layer.getCrossPointRangeDistCost({0, layer.numCrossPoints() - 1});
    }
    log() << "The total number of grid points is " << numGridPoints << std::endl;
    log() << "The total length of tracks is " << totalTrackLength << " DBU " << std::endl;
    log() << "(Note: dir=Y means that each track is horizontal and has differnt Y)" << std::endl;
    log() << "Metal layer design rules: " << std::endl;
    for (const MetalLayer& layer : layers) {
        layer.printDesignRules(log()) << std::endl;
    }
    log() << "Wire conflict LUT: " << std::endl;
    for (const MetalLayer& layer : layers) {
        layer.printViaOccupancyLUT(log()) << std::endl;
    }

    log() << "CUT LAYERS" << std::endl;
    numVias = 0;
    for (int i = 0; i < getLayerNum() - 1; ++i) {
        int num = layers[i].numTracks() * layers[i + 1].numTracks();
        const auto& cutLayer = cutLayers[i];
        log() << cutLayer << ", #vias=" << num << std::endl;
        numVias += num;
    }
    log() << "Cut layer design rules: " << std::endl;
    for (const CutLayer& layer : cutLayers) {
        layer.printDesignRules(log()) << std::endl;
    }
    log() << "Via conflict LUT: " << std::endl;
    for (const CutLayer& layer : cutLayers) {
        layer.printViaOccupancyLUT(log()) << std::endl;
    }
    log() << "The total number of via candidates is " << numVias << std::endl;
}

void LayerList::writeDefConflictLUTs(const std::string& debugFileName) const {
    std::ofstream ofs(debugFileName);
    int cpIdx = 0;
    for (const auto& cutLayer : cutLayers) {
        cutLayer.printBasics(ofs);
        cutLayer.printDesignRules(ofs);
        cutLayer.printViaOccupancyLUT(ofs);
        ofs << "viaCut" << std::endl;
        int xSize = ((int)cutLayer.allViaTypes[0].allViaCut[0].size() - 1) / 2;
        int ySize = ((int)cutLayer.allViaTypes[0].allViaCut[0][0].size() - 1) / 2;
        for (int j = xSize; j != cutLayer.allViaTypes[0].allViaCut[0].size(); ++j) {
            for (int k = ySize; k != cutLayer.allViaTypes[0].allViaCut[0][0].size(); ++k) {
                ofs << (int)(cutLayer.allViaTypes[0].allViaCut[0][j][k]) << " ";
            }
            ofs << std::endl;
        }
        ofs << std::endl;
        ofs << "viaMetal" << std::endl;
        xSize = ((int)cutLayer.allViaTypes[0].allViaMetal[0].size() - 1) / 2;
        ySize = ((int)cutLayer.allViaTypes[0].allViaMetal[0][0].size() - 1) / 2;
        for (int j = xSize; j != cutLayer.allViaTypes[0].allViaMetal[0].size(); ++j) {
            for (int k = ySize; k != cutLayer.allViaTypes[0].allViaMetal[0][0].size(); ++k) {
                ofs << (int)(cutLayer.allViaTypes[0].allViaMetal[0][j][k]) << " ";
            }
            ofs << std::endl;
        }
        ofs << std::endl;
        ofs << "viaBotVia" << std::endl;
        cpIdx = 0;
        if (cutLayer.idx > 0) {
            for (const auto& cp : cutLayer.allViaTypes[0].allViaBotVia[0]) {
                ofs << "cpidx is: " << cpIdx++ << std::endl;
                for (auto a : cp) {
                    for (auto b : a) {
                        ofs << (int)(b) << " ";
                    }
                    ofs << std::endl;
                }
            }
        }
        ofs << "viaTopVia" << std::endl;
        cpIdx = 0;
        if (cutLayer.idx < cutLayers.size() - 1) {
            for (const auto& cp : cutLayer.allViaTypes[0].allViaTopVia[0]) {
                ofs << "cpidx is: " << cpIdx++ << std::endl;
                for (auto a : cp) {
                    for (auto b : a) {
                        ofs << (int)(b) << " ";
                    }
                    ofs << std::endl;
                }
            }
        }
        ofs << std::endl;
    }
}

}  // namespace db
