#include "CongestionMap.h"
#include "global.h"

double CongestionMap::calcCrsnEdgeCost(const gr::PointOnLayer& u, const gr::PointOnLayer& v) {
    if (u.layerIdx != v.layerIdx && abs(u[X] - v[X]) + abs(u[Y] - v[Y]) != 1) {
        printlog("Warning: CongestionMap::calcCrsnEdgeCost,", "edge longer than one,", u, v);
        return std::numeric_limits<double>::max();
    }

    int layerIdx = u.layerIdx;
    auto layerDir = database.getLayerDir(layerIdx);

    if (layerIdx == 0) return LARGE_NUM;

    utils::BoxT<int> box;
    auto u_box = getGrBox(u);
    auto v_box = getGrBox(v);

    double cost = 0;
    box = u_box;
    double avgRsrc = 0;
    for (int x_idx = box.x.low; x_idx <= box.x.high; x_idx++) {
        for (int y_idx = box.y.low; y_idx <= box.y.high; y_idx++) {
            avgRsrc += rsrcMap[layerIdx][x_idx][y_idx];
        }
    }
    avgRsrc /= (box.x.range() + 1) * (box.y.range() + 1);
    cost += double(layerDir == X ? yCrsnScale : xCrsnScale) * 1.0 / max(avgRsrc, 0.1);

    box = v_box;
    avgRsrc = 0;
    for (int x_idx = box.x.low; x_idx <= box.x.high; x_idx++) {
        for (int y_idx = box.y.low; y_idx <= box.y.high; y_idx++) {
            avgRsrc += rsrcMap[layerIdx][x_idx][y_idx];
        }
    }
    avgRsrc /= (box.x.range() + 1) * (box.y.range() + 1);
    cost += double(layerDir == X ? yCrsnScale : xCrsnScale) * 1.0 / max(avgRsrc, 0.1);

    return cost;
}

double CongestionMap::calcCrsnViaCost(const gr::PointOnLayer& via) {
    utils::BoxT<int> box = getGrBox(via);
    double cost = 0;
    for (int l_idx = via.layerIdx; l_idx <= via.layerIdx + 1; l_idx++) {
        double avgRsrc = 0;
        for (int x_idx = box.x.low; x_idx <= box.x.high; x_idx++) {
            for (int y_idx = box.y.low; y_idx <= box.y.high; y_idx++) {
                avgRsrc += rsrcMap[l_idx][x_idx][y_idx];
            }
        }
        avgRsrc /= (box.x.range() + 1) * (box.y.range() + 1);
        cost += 1.0 / max(avgRsrc, 0.1);
    }

    return cost;
}

double CongestionMap::getCrsnEdgeCost(const gr::PointOnLayer& u, const gr::PointOnLayer& v) const {
    if (u.layerIdx != v.layerIdx && abs(u[X] - v[X]) + abs(u[Y] - v[Y]) != 1) {
        printlog("Warning: CongestionMap::calcCrsnEdgeCost,", "edge longer than one,", u, v);
        return std::numeric_limits<double>::max();
    }

    int layerIdx = u.layerIdx;
    auto layerDir = database.getLayerDir(layerIdx);
    int gridline = u[layerDir];
    int cp = min(u[1 - layerDir], v[1 - layerDir]);

    return crsnEdgeCostMap[layerIdx][gridline][cp];
}

double CongestionMap::getCrsnViaCost(const gr::PointOnLayer& via) const {
    return crsnViaCostMap[via.layerIdx][via.x][via.y];
}

utils::BoxT<int> CongestionMap::getGrBox(const gr::PointOnLayer& u) const {  // get the gcell box in a coarsened cell
    utils::BoxT<int> gcellBox;
    gcellBox.x.Update(u[X] * xCrsnScale);
    gcellBox.x.Update(min(u[X] * xCrsnScale + xCrsnScale, grDatabase.getNumGrPoint(X)) - 1);
    gcellBox.y.Update(u[Y] * yCrsnScale);
    gcellBox.y.Update(min(u[Y] * yCrsnScale + yCrsnScale, grDatabase.getNumGrPoint(Y)) - 1);
    return gcellBox;
}

gr::PointOnLayer CongestionMap::grPointToPoint(const gr::GrPoint& point) const {
    return {point.layerIdx, point[X] / xCrsnScale, point[Y] / yCrsnScale};
}

void CongestionMap::init(int x_crsn_scale, int y_crsn_scale) {
    xCrsnScale = x_crsn_scale;
    yCrsnScale = y_crsn_scale;

    rsrcMap.assign(database.getLayerNum(),
                   vector<vector<double>>(grDatabase.getNumGrPoint(X), vector<double>(grDatabase.getNumGrPoint(Y), 0)));

    for (int l_idx = 0; l_idx < database.getLayerNum(); l_idx++)
        for (int x_idx = 0; x_idx < grDatabase.getNumGrPoint(X); x_idx++)
            for (int y_idx = 0; y_idx < grDatabase.getNumGrPoint(Y); y_idx++)
                rsrcMap[l_idx][x_idx][y_idx] = grDatabase.getCellUsage({l_idx, x_idx, y_idx});

    xNumCrsnCell = ceil(grDatabase.getNumGrPoint(X) / (double)xCrsnScale);
    yNumCrsnCell = ceil(grDatabase.getNumGrPoint(Y) / (double)xCrsnScale);

    crsnViaCostMap.clear();
    crsnEdgeCostMap.clear();
    crsnViaCostMap.resize(database.getLayerNum() - 1,
                          vector<vector<double>>(xNumCrsnCell, vector<double>(yNumCrsnCell)));
    crsnEdgeCostMap.resize(database.getLayerNum());
    for (int l = 0; l < database.getLayerNum(); l++) {
        if (database.getLayerDir(l) == X)
            crsnEdgeCostMap[l].resize(xNumCrsnCell, vector<double>(yNumCrsnCell - 1));
        else
            crsnEdgeCostMap[l].resize(yNumCrsnCell, vector<double>(xNumCrsnCell - 1));
    }
    for (int l = 0; l < database.getLayerNum(); l++) {
        if (database.getLayerDir(l) == X) {
            for (int gridline = 0; gridline < xNumCrsnCell; gridline++)
                for (int cp = 0; cp < yNumCrsnCell - 1; cp++)
                    crsnEdgeCostMap[l][gridline][cp] = calcCrsnEdgeCost({l, gridline, cp}, {l, gridline, cp + 1});
        } else {
            for (int gridline = 0; gridline < yNumCrsnCell; gridline++)
                for (int cp = 0; cp < xNumCrsnCell - 1; cp++)
                    crsnEdgeCostMap[l][gridline][cp] = calcCrsnEdgeCost({l, cp, gridline}, {l, cp + 1, gridline});
        }
    }

    for (int l = 0; l < database.getLayerNum() - 1; l++) {
        for (int x = 0; x < xNumCrsnCell; x++)
            for (int y = 0; y < yNumCrsnCell; y++) crsnViaCostMap[l][x][y] = calcCrsnViaCost({l, x, y});
    }
}

void CongestionMap::update(const gr::GrNet& net) {
    std::unordered_set<gr::PointOnLayer> pointSet;

    auto updateBox = [&](const gr::GrBoxOnLayer& box) {
        int l_idx = box.layerIdx;
        for (int x_idx = box[X].low; x_idx <= box[X].high; x_idx++) {
            for (int y_idx = box[Y].low; y_idx <= box[Y].high; y_idx++) {
                rsrcMap[l_idx][x_idx][y_idx] = grDatabase.getCellUsage({l_idx, x_idx, y_idx});
                pointSet.insert(grPointToPoint({l_idx, x_idx, y_idx}));
            }
        }
    };

    for (auto& guide : net.wireRouteGuides) {
        // enlarge the updating box
        auto box = guide;
        if (database.getLayerDir(guide.layerIdx) == X) {
            box[Y].Update(max(guide[Y].low - 1, 0));
            box[Y].Update(min(guide[Y].high + 1, grDatabase.getNumGrPoint(Y) - 1));
        } else {
            box[X].Update(max(guide[X].low - 1, 0));
            box[X].Update(min(guide[X].high + 1, grDatabase.getNumGrPoint(X) - 1));
        }
        updateBox(box);
    }
    for (auto& guide : net.viaRouteGuides) {
        updateBox(guide);
    }

    for (auto& point : pointSet) {
        updateViaCostMap(point);
        updateWireCostMap(point);
    }
}

void CongestionMap::updateViaCostMap(const gr::PointOnLayer& point) {
    int layerIdx = point.layerIdx;

    if (layerIdx < database.getLayerNum() - 1) crsnViaCostMap[layerIdx][point.x][point.y] = calcCrsnViaCost(point);
    if (layerIdx > 0)
        crsnViaCostMap[layerIdx - 1][point.x][point.y] = calcCrsnViaCost({layerIdx - 1, point.x, point.y});
}

void CongestionMap::updateWireCostMap(const gr::PointOnLayer& point) {
    int layerIdx = point.layerIdx;
    auto layerDir = database.getLayerDir(layerIdx);
    int gridline = point[layerDir];
    int cp = point[1 - layerDir];
    int cpNum = (layerDir == X) ? yNumCrsnCell : xNumCrsnCell;

    if (layerDir == X) {
        if (cp > 0)
            crsnEdgeCostMap[layerIdx][gridline][cp - 1] =
                calcCrsnEdgeCost({layerIdx, point.x, point.y - 1}, {layerIdx, point.x, point.y});
        if (cp < cpNum - 1)
            crsnEdgeCostMap[layerIdx][gridline][cp] =
                calcCrsnEdgeCost({layerIdx, point.x, point.y}, {layerIdx, point.x, point.y + 1});
    } else {
        if (cp > 0)
            crsnEdgeCostMap[layerIdx][gridline][cp - 1] =
                calcCrsnEdgeCost({layerIdx, point.x - 1, point.y}, {layerIdx, point.x, point.y});
        if (cp < cpNum - 1)
            crsnEdgeCostMap[layerIdx][gridline][cp] =
                calcCrsnEdgeCost({layerIdx, point.x, point.y}, {layerIdx, point.x + 1, point.y});
    }
}
