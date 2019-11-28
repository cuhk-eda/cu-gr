#include "GrNet.h"
#include "GrDatabase.h"

namespace gr {

void GrNet::init(const GCellGrid& gcellGrid) {
    initPinAccessBoxes(gcellGrid);

    // get overlap points
    for (int p1 = 0; p1 < numOfPins(); p1++) {
        for (int p2 = p1 + 1; p2 < numOfPins(); p2++) {
            for (const auto& point1 : pinAccessBoxes[p1]) {
                for (const auto& point2 : pinAccessBoxes[p2]) {
                    if (point1 == point2) ovlpPoints.insert(point1);
                }
            }
        }
    }

    // judge if it is one pin net
    std::unordered_map<gr::GrPoint, int> pointSet;
    for (const auto& pin : pinAccessBoxes) {
        for (const auto& point : pin) pointSet[point]++;
    }

    for (const auto& pair : pointSet) {
        if (pair.second == numOfPins()) {
            isOnePin = true;
            break;
        }
    }

    for (const auto& pinBox : pinAccessBoxes) {
        for (const auto& grpnt : pinBox) {
            boundingBox[X].Update(grpnt[X]);
            boundingBox[Y].Update(grpnt[Y]);
        }
    }
}

// merged pins with same coor
vector<vector<PointOnLayer>> GrNet::getMergedPinAccessBoxes(
    const std::function<PointOnLayer(GrPoint)>& pointHash) const {
    vector<vector<PointOnLayer>> mergedPinAccessBoxes;

    vector<vector<PointOnLayer>> hashedPinAccessBoxes(numOfPins());
    for (int p = 0; p < numOfPins(); p++)
        for (const auto& point : pinAccessBoxes[p]) hashedPinAccessBoxes[p].push_back(pointHash(point));

    vector<vector<int>> pinConn(numOfPins());
    for (int p1 = 0; p1 < numOfPins(); p1++) {
        for (int p2 = p1 + 1; p2 < numOfPins(); p2++) {
            bool overlap = false;
            for (const auto& point1 : hashedPinAccessBoxes[p1]) {
                if (overlap) break;
                for (const auto& point2 : hashedPinAccessBoxes[p2]) {
                    if (overlap) break;
                    if (point1 == point2) {
                        pinConn[p1].push_back(p2);
                        pinConn[p2].push_back(p1);
                        overlap = true;
                    }
                }
            }
        }
    }
    vector<bool> visited(numOfPins(), false);
    for (int p = 0; p < numOfPins(); p++) {
        if (visited[p]) continue;
        std::queue<int> q;
        q.push(p);
        vector<PointOnLayer> points;
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            copy(hashedPinAccessBoxes[node].begin(), hashedPinAccessBoxes[node].end(), std::back_inserter(points));
            visited[node] = true;
            for (auto child : pinConn[node])
                if (!visited[child]) q.push(child);
        }
        mergedPinAccessBoxes.resize(mergedPinAccessBoxes.size() + 1);
        mergedPinAccessBoxes.back() = move(points);
    }
    for (auto& points : mergedPinAccessBoxes) {
        std::sort(points.begin(), points.end(), [&](const PointOnLayer& lhs, const PointOnLayer& rhs) {
            if (lhs.layerIdx != rhs.layerIdx) {
                return lhs.layerIdx < rhs.layerIdx;
            } else {
                if (lhs[X] != rhs[X]) {
                    return lhs[X] < rhs[X];
                } else {
                    return lhs[Y] < rhs[Y];
                }
            }
        });
        points.erase(std::unique(points.begin(), points.end()), points.end());
    }

    return mergedPinAccessBoxes;
}

void GrNet::initPinAccessBoxes(const GCellGrid& gcellGrid) {
    // transform coor to grPoint, construct pinAccessBoxes
    pinAccessBoxes.resize(numOfPins());
    for (int i = 0; i < numOfPins(); i++) {
        const auto& boxes = dbNet.pinAccessBoxes[i];
        std::unordered_set<GrPoint> pointSet;

        DBU smallestVio = std::numeric_limits<DBU>::max();
        vector<const db::BoxOnLayer*> smallestBoxes;

        for (const auto& box : boxes) {
            int vio = database.getOvlpFixedMetalArea(box, dbNet.idx);
            if (vio <= smallestVio) {
                if (vio < smallestVio) smallestBoxes.clear();
                smallestVio = vio;
                smallestBoxes.push_back(&box);
            }

            if (vio == 0) {
                auto grBox = gcellGrid.rangeSearchGCell(box);
                for (int x = grBox[X].low; x <= grBox[X].high; x++)
                    for (int y = grBox[Y].low; y <= grBox[Y].high; y++) pointSet.emplace(box.layerIdx, x, y);
            }
        }
        // all have vio, add those with smallest vio
        if (pointSet.empty()) {
            for (auto box : smallestBoxes) {
                auto grBox = gcellGrid.rangeSearchGCell(*box);
                for (int x = grBox[X].low; x <= grBox[X].high; x++)
                    for (int y = grBox[Y].low; y <= grBox[Y].high; y++) pointSet.emplace(box->layerIdx, x, y);
            }
        }

        for (auto& point : pointSet) pinAccessBoxes[i].push_back(point);
    }
}

void GrNetlist::init(const GCellGrid& gcellGrid) {
    for (int i = 0, sz = database.nets.size(); i < sz; i++) nets.emplace_back(i);
    runJobsMT(database.nets.size(), [&](int i) { nets[i].init(gcellGrid); });
}

void GrNet::postOrderVisitGridTopo(const std::function<void(std::shared_ptr<GrSteiner>)>& visit) const {
    for (const std::shared_ptr<GrSteiner>& tree : gridTopo) {
        GrSteiner::postOrder(tree, visit);
    }
}

void GrNet::preOrderVisitGridTopo(const std::function<void(std::shared_ptr<GrSteiner>)>& visit) const {
    for (const std::shared_ptr<GrSteiner>& tree : gridTopo) {
        GrSteiner::preOrder(tree, visit);
    }
}

DBU GrNet::getWirelength() const {
    DBU wirelength = 0;
    postOrderVisitGridTopo([&](std::shared_ptr<gr::GrSteiner> node) {
        auto parent = node;
        for (auto child : parent->children) {
            if (parent->layerIdx == child->layerIdx) wirelength += grDatabase.getDist(*parent, *child);
        }
    });
    return wirelength;
}

}  // namespace gr