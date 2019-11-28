#include "GridGraph.h"
#include "global.h"
#include "GenGuide.h"

void GridGraph::init(int nNodes) {
    conn.resize(nNodes, {-1, -1, -1, -1});
    edgeCost.resize(nNodes, {-1, -1, -1, -1});

    edgeCount = 0;
}

void GridGraph::addEdge(int u, int v, EdgeDirection dir, db::CostT w) {
    if (hasEdge(u, dir)) return;

    edgeCost[u][dir] = edgeCost[v][getOppDir(dir)] = w;
    conn[u][dir] = v;
    conn[v][oppDirections[dir]] = u;

    edgeCount++;
}

void GridGraph::writeDebugFile(const std::string &fn) const {
    std::ofstream debugFile(fn);
    for (int i = 0; i < conn.size(); ++i) {
        debugFile << i << " " << vertexToPoint[i] << " edgeC=" << edgeCost[i] << " conn=" << conn[i] << std::endl;
    }
}

bool GridGraph::checkConn() const {
    auto BFS = [&](int s, vector<bool> &visited) {
        std::queue<int> q;
        visited[s] = true;
        q.push(s);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (auto v : conn[u]) {
                if (v != -1 && !visited[v]) {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }
    };

    vector<bool> visited(conn.size(), false);
    BFS(0, visited);
    for (int i = 0; i < visited.size(); i++)
        if (!visited[i]) return false;
    return true;
}

bool switchLayer(EdgeDirection direction) { return direction == UP || direction == DOWN; }

EdgeDirection getOppDir(EdgeDirection direction) { return oppDirections[direction]; }

void CoarseGridGraphBuilder::run(const vector<vector<gr::PointOnLayer>> &mergedPinAccessBoxes) {
    numPointsX = ceil(grDatabase.getNumGrPoint(X) / (double)cellWidth);
    numPointsY = ceil(grDatabase.getNumGrPoint(Y) / (double)cellHeight);
    int numPoints = numPointsX * numPointsY * database.getLayerNum();

    // 1. Give each grid point an index
    graph.vertexToPoint.reserve(numPoints);
    for (int l = 0; l < database.getLayerNum(); l++)
        for (int x = 0; x < numPointsX; x++)
            for (int y = 0; y < numPointsY; y++) graph.vertexToPoint.emplace_back(l, x, y);

    // 2. Add vertex-pin connection
    graph.pinToVertex.resize(mergedPinAccessBoxes.size());
    for (unsigned p = 0; p < mergedPinAccessBoxes.size(); p++) {
        for (const auto &point : mergedPinAccessBoxes[p]) {
            int u = pointToVertex(point);
            graph.pinToVertex[p].push_back(u);
            graph.vertexToPin[u] = p;
        }
    }

    graph.init(numPoints);

    // 3. Add inter-layer connection
    for (int l = 0; l < database.getLayerNum() - 1; l++) {
        int lowerBias = numPointsX * numPointsY * l;
        int upperBias = numPointsX * numPointsY * (l + 1);
        for (int x = 0; x < numPointsX; x++) {
            int bias = numPointsY * x;
            for (int y = 0; y < numPointsY; y++) {
                int u = lowerBias + bias + y;
                int v = upperBias + bias + y;
                graph.addEdge(u, v, UP, congMap.getCrsnViaCost({l, x, y}));
            }
        }
    }

    // 4. add intra-layer connection
    for (int l = 0; l < database.getLayerNum(); l++) {
        int layerBias = numPointsX * numPointsY * l;
        if (database.getLayerDir(l) == X) {
            for (int x = 0; x < numPointsX; x++) {
                int bias = numPointsY * x;
                for (int y = 0; y < numPointsY - 1; y++) {
                    int u = layerBias + bias + y;
                    int v = layerBias + bias + y + 1;
                    graph.addEdge(u, v, FORWARD, congMap.getCrsnEdgeCost({l, x, y}, {l, x, y + 1}));
                }
            }
        } else {
            for (int x = 0; x < numPointsX - 1; x++) {
                int bias = numPointsY * x;
                for (int y = 0; y < numPointsY; y++) {
                    int u = layerBias + bias + y;
                    int v = layerBias + bias + y + numPointsY;
                    graph.addEdge(u, v, FORWARD, congMap.getCrsnEdgeCost({l, x, y}, {l, x + 1, y}));
                }
            }
        }
    }
}

int CoarseGridGraphBuilder::pointToVertex(const gr::PointOnLayer &point) const {
    return numPointsX * numPointsY * point.layerIdx + numPointsY * point[X] + point[Y];
}

gr::PointOnLayer CoarseGridGraphBuilder::grPointToPoint(const gr::GrPoint &point) const {
    return congMap.grPointToPoint(point);
}

double CoarseGridGraphBuilder::getCost(int u, int v) { return 0; }

void GuideGridGraphBuilder::run(const vector<vector<gr::PointOnLayer>> &mergedPinAccessBoxes) {
    // 0. slice guides by their prefer direction
    GuideGenerator::sliceGuides(guides);

    // 1. Give each grid point an index
    for (auto &box : guides) {
        int begin = intervals.empty() ? 0 : intervals.back().second;
        int end = begin + (box[X].range() + 1) * (box[Y].range() + 1);

        intervals.emplace_back(begin, end);
    }
    graph.vertexToPoint.reserve(intervals.back().second);
    for (auto &box : guides) {
        auto dir = database.getLayerDir(box.layerIdx);
        for (int gridline = box[dir].low; gridline <= box[dir].high; gridline++) {
            for (int cp = box[1 - dir].low; cp <= box[1 - dir].high; cp++) {
                if (dir == X)
                    graph.vertexToPoint.emplace_back(box.layerIdx, gridline, cp);
                else
                    graph.vertexToPoint.emplace_back(box.layerIdx, cp, gridline);
            }
        }
    }

    // 2. Add guide-pin connection
    graph.pinToVertex.resize(mergedPinAccessBoxes.size());
    for (unsigned p = 0; p < mergedPinAccessBoxes.size(); p++) {
        for (const auto &point : mergedPinAccessBoxes[p]) {
            auto dir = database.getLayerDir(point.layerIdx);
            for (unsigned g = 0; g < guides.size(); g++) {
                if (guides[g].includePoint({point.layerIdx, point[X], point[Y]})) {
                    int u = guideToVertex(g, point[dir], point[1 - dir]);
                    graph.pinToVertex[p].push_back(u);
                    graph.vertexToPin[u] = p;
                }
            }
        }
    }

    graph.init(intervals.back().second);

    // 3. Add inter-guide connection
    for (unsigned b1 = 0; b1 < guides.size(); b1++) {
        for (unsigned b2 = b1 + 1; b2 < guides.size(); b2++)
            if (guides[b1][X].HasIntersectWith(guides[b2][X]) && guides[b1][Y].HasIntersectWith(guides[b2][Y]) &&
                abs(guides[b1].layerIdx - guides[b2].layerIdx) <= 1)
                connectTwoGuides(b1, b2);
    }

    // 4. add intra-guide connection
    for (unsigned b = 0; b < guides.size(); b++) connectGuide(b);
}

void GuideGridGraphBuilder::connectGuide(int guideIdx) {
    const auto &box = guides[guideIdx];

    int layerIdx = box.layerIdx;
    auto dir = database.getLayerDir(layerIdx);

    const auto &gridlineRange = box[dir];
    const auto &cpRange = box[1 - dir];
    int netIdx = grNet.dbNet.idx;

    auto setEdgeCost = [&](int gridline, int beginCP, int endCP) {
        if (beginCP == endCP) return;

        int u = guideToVertex(guideIdx, gridline, beginCP);
        int v = guideToVertex(guideIdx, gridline, endCP);

        gr::GrPoint point1(layerIdx, dir == X ? gridline : beginCP, dir == X ? beginCP : gridline);
        gr::GrPoint point2(layerIdx, dir == X ? gridline : endCP, dir == X ? endCP : gridline);
        db::CostT cost = grDatabase.getWireCost({point1, point2});

        graph.addEdge(u, v, FORWARD, cost);
    };

    auto canRemove = [&](int gridline, int cp) {
        int vertexIdx = guideToVertex(guideIdx, gridline, cp);
        return !graph.hasEdge(vertexIdx, UP) && !graph.hasEdge(vertexIdx, DOWN) && graph.getPinIdx(vertexIdx) == -1 &&
               cp != cpRange.low && cp != cpRange.high;
    };

    if (cpRange.range() == 0) return;

    for (int g = gridlineRange.low; g <= gridlineRange.high; g++) {
        // get the directIntervals
        vector<utils::IntervalT<int>> directIntervals;
        int begin = -1, end = -1;
        for (int c = cpRange.low; c <= cpRange.high; c++) {
            if (begin == -1) {
                if (canRemove(g, c)) begin = c - 1;
            } else {
                if (!canRemove(g, c)) {
                    end = c;
                    directIntervals.emplace_back(begin, end);
                    begin = -1;
                    end = -1;
                }
            }
        }

        // get the indirectIntervals
        vector<utils::IntervalT<int>> indirectIntervals;
        if (directIntervals.empty()) {
            indirectIntervals.push_back(cpRange);
        } else {
            indirectIntervals.emplace_back(cpRange.low, directIntervals.begin()->low);
            for (int i = 0; i + 1 < directIntervals.size(); i++) {
                indirectIntervals.emplace_back(directIntervals[i].high, directIntervals[i + 1].low);
            }
            indirectIntervals.emplace_back(directIntervals.rbegin()->high, cpRange.high);
        }

        // connect the intervals
        for (auto &interval : directIntervals) setEdgeCost(g, interval.low, interval.high);

        for (auto &interval : indirectIntervals)
            for (int c = interval.low; c + 1 <= interval.high; c++) setEdgeCost(g, c, c + 1);
    }
}

void GuideGridGraphBuilder::connectTwoGuides(int guideIdx1, int guideIdx2) {
    const auto &box1 = guides[guideIdx1];
    const auto &box2 = guides[guideIdx2];

    int upperIdx = box1.layerIdx > box2.layerIdx ? guideIdx1 : guideIdx2;
    int lowerIdx = box1.layerIdx < box2.layerIdx ? guideIdx1 : guideIdx2;

    auto lowerDir = database.getLayerDir(guides[lowerIdx].layerIdx);

    auto lowerGridLineIntvl = guides[lowerIdx][lowerDir].IntersectWith(guides[upperIdx][lowerDir]);
    auto lowerCPIntvl = guides[lowerIdx][1 - lowerDir].IntersectWith(guides[upperIdx][1 - lowerDir]);

    for (int lowerGridLine = lowerGridLineIntvl.low; lowerGridLine <= lowerGridLineIntvl.high; lowerGridLine++) {
        for (int lowerCPIdx = lowerCPIntvl.low; lowerCPIdx <= lowerCPIntvl.high; lowerCPIdx++) {
            int u = guideToVertex(upperIdx, lowerCPIdx, lowerGridLine);
            int v = guideToVertex(lowerIdx, lowerGridLine, lowerCPIdx);

            db::CostT cost = grDatabase.getViaCost({guides[lowerIdx].layerIdx,
                                                    lowerDir == X ? lowerGridLine : lowerCPIdx,
                                                    lowerDir == X ? lowerCPIdx : lowerGridLine});

            graph.addEdge(u, v, DOWN, cost);
        }
    }
}

int GuideGridGraphBuilder::guideToVertex(int gIdx, int gridline, int cp) const {
    const auto &box = guides[gIdx];
    int pointBias = intervals[gIdx].first;

    return boxToVertex(box, pointBias, gridline, cp);
}

int GuideGridGraphBuilder::boxToVertex(const gr::GrBoxOnLayer &box, int pointBias, int gridline, int cp) const {
    auto dir = database.getLayerDir(box.layerIdx);
    return pointBias + (gridline - box[dir].low) * (box[1 - dir].range() + 1) + (cp - box[1 - dir].low);
}
