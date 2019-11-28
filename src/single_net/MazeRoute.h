#pragma once

#include "GridGraph.h"
#include "multi_net/CongestionMap.h"

class Solution {
public:
    db::CostT cost;
    int vertex;
    std::shared_ptr<Solution> prev;

    Solution(db::CostT c, int v, const std::shared_ptr<Solution> &p) : cost(c), vertex(v), prev(p) {}

    friend ostream &operator<<(ostream &os, const Solution &sol);
};

class MazeRoute {
public:
    MazeRoute(gr::GrNet &grNetData) : grNet(grNetData) {}

    void constructGridGraph(const vector<gr::GrBoxOnLayer> &guides);
    void constructGridGraph(const CongestionMap& congMap);
    db::RouteStatus run();

private:
    gr::GrNet &grNet;
    GridGraph graph;

    vector<db::CostT> vertexCosts;              // min cost upper bound for each vertex
    vector<std::shared_ptr<Solution>> pinSols;  // best solution for each pin
    vector<vector<gr::PointOnLayer>> mergedPinAccessBoxes;

    db::RouteStatus route(int startPin);
    void getResult();
};
