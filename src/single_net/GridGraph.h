#pragma once
#include "global.h"
#include "db/Database.h"
#include "gr_db/GrDatabase.h"
#include "multi_net/CongestionMap.h"

enum EdgeDirection { BACKWARD = 0, FORWARD = 1, UP = 2, DOWN = 3 };

const vector<EdgeDirection> directions = {BACKWARD, FORWARD, UP, DOWN};
const vector<EdgeDirection> oppDirections = {FORWARD, BACKWARD, DOWN, UP};

bool switchLayer(EdgeDirection direction);
EdgeDirection getOppDir(EdgeDirection direction);

class GuideGridGraphBuilder;
class CoarseGridGraphBuilder;

// Note: GridGraph will be across both GridGraphBuilder & MazeRoute
class GridGraph {
public:
    friend GuideGridGraphBuilder;
    friend CoarseGridGraphBuilder;

    // getters
    bool hasEdge(int u, EdgeDirection dir) const { return conn[u][dir] != -1; }
    int getEdgeEndPoint(int u, EdgeDirection dir) const { return conn[u][dir]; }
    db::CostT getEdgeCost(int u, EdgeDirection dir) const { return edgeCost[u][dir]; }
    gr::PointOnLayer& getPoint(int u) { return vertexToPoint[u]; }
    int getEdgeNum() const { return edgeCount; }
    int getNodeNum() const { return conn.size(); }
    int getPinIdx(int u) const {
        auto it = vertexToPin.find(u);
        return (it != vertexToPin.end()) ? it->second : -1;
    }
    vector<int>& getVertices(int pinIdx) { return pinToVertex[pinIdx]; }

    void writeDebugFile(const std::string& fn) const;
    bool checkConn() const;

private:
    int edgeCount;

    // vertex properties
    std::unordered_map<int, int> vertexToPin;  // vertexIdx to pinIdx
    vector<vector<int>> pinToVertex;
    vector<gr::PointOnLayer> vertexToPoint;

    // adj lists
    vector<std::array<int, 4>> conn;
    vector<std::array<db::CostT, 4>> edgeCost;

    // setters
    void init(int nNodes);
    void addEdge(int u, int v, EdgeDirection dir, db::CostT w);
};

class GridGraphBuilderBase {
public:
    GridGraphBuilderBase(gr::GrNet& grNetData, GridGraph& graphData) : grNet(grNetData), graph(graphData){};

    virtual void run(const vector<vector<gr::PointOnLayer>>& mergedPinAccessBoxes) = 0;

protected:
    gr::GrNet& grNet;
    GridGraph& graph;
};

class GuideGridGraphBuilder : public GridGraphBuilderBase {
public:
    GuideGridGraphBuilder(gr::GrNet& grNetData, GridGraph& graphData, const vector<gr::GrBoxOnLayer>& guidesData)
        : GridGraphBuilderBase(grNetData, graphData), guides(guidesData) {}

    virtual void run(const vector<vector<gr::PointOnLayer>>& mergedPinAccessBoxes);

private:
    vector<gr::GrBoxOnLayer> guides;
    vector<std::pair<int, int>> intervals;

    void connectGuide(int guideIdx);
    void connectTwoGuides(int guideIdx1, int guideIdx2);

    int guideToVertex(int gIdx, int x, int y) const;
    int boxToVertex(const gr::GrBoxOnLayer& box, int pointBias, int x, int y) const;
};

class CoarseGridGraphBuilder : public GridGraphBuilderBase {
public:
    CoarseGridGraphBuilder(gr::GrNet& grNetData, GridGraph& graphData, const CongestionMap& congMapData)
        : GridGraphBuilderBase(grNetData, graphData),
          congMap(congMapData),
          cellWidth(congMap.getCellWidth()),
          cellHeight(congMap.getCellHeight()) {}

    virtual void run(const vector<vector<gr::PointOnLayer>>& mergedPinAccessBoxes);
    gr::PointOnLayer grPointToPoint(const gr::GrPoint& point) const;

private:
    const CongestionMap& congMap;
    int cellWidth;
    int cellHeight;
    int numPointsX;
    int numPointsY;

    int pointToVertex(const gr::PointOnLayer& point) const;
    double getCost(int u, int v);
};