#pragma once

#include "global.h"
#include "flute/flute.h"
#include "gr_db/GrDatabase.h"
#include "GenGuide.h"
#include <map>

extern "C" {
Tree flute(int d, DTYPE x[], DTYPE y[], int acc);
}

class RouteNode {
public:
    int x;
    int y;
    int idx;
    vector<int> pinLayers;    // pins' layers within the node
    vector<int> pinIdxs;      // pins' indexes within the node
    std::set<int> toConnect;  // nodes connecting to this one
    vector<int> childIdxs;

    // PatternRoute
    vector<db::CostT> exitCosts;  // cost exiting the node from different layers
    // cost of entering the node on different layers from different children
    std::unordered_map<int, vector<db::CostT>> enterCosts;  // (childIdx->layerIdx->cost)
    // topo of entering the node on different layers from different children
    std::unordered_map<int, vector<vector<gr::GrPoint>>> enterEdges;  // (childIdx->layerIdx->edge)
    // which layer should a child edge enter if exiting from a specific layer
    std::unordered_map<int, vector<int>> exitEnterLayers;  // (childIdx->exit layer-> enter layer)

    int degree() { return toConnect.size(); }

    std::string str() {  // return string description
        return std::to_string(idx) + "(" + std::to_string(x) + ", " + std::to_string(y) + ")" +
               (pinIdxs.size() == 0 ? "Steiner" : " ");
    }

    RouteNode() : x(-1), y(-1) {}
    RouteNode(int nx, int ny) : x(nx), y(ny) {}
    RouteNode(std::tuple<int, int> loc) : x(std::get<0>(loc)), y(std::get<1>(loc)) {}
};

struct RouteEdge {
    int from;
    int to;
};

class InitRoute {
public:
    InitRoute(gr::GrNet &grNetData) : grNet(grNetData), guideGen(grNet), status(db::RouteStatus::SUCC_NORMAL) {
        grNet.gridTopo.clear();
    }

    void patternRoute();  // pattern routing
    void buildTopo();

    void plan_fluteOnly();
    void edge_shift2d(std::map<int, RouteNode> &routeNodes);
    void getRoutingOrder();
    void addUsage2D(RouteNode &u, RouteNode &v, double usage = 1);
    void removeUsage2D(RouteNode &u, RouteNode &v, double usage = 1);

    std::map<int, RouteNode> &getRouteNodes();

    db::RouteStatus status;
    gr::GrNet &grNet;

private:
    GuideGenerator guideGen;
    std::map<int, RouteNode> routeNodes;
    vector<RouteEdge> routeEdges;

    db::CostT getBufferedWireCost(gr::GrEdge edge);
    std::unordered_map<gr::GrEdge, db::CostT> wireCostBuffer;

    void LShape(const RouteEdge &edge);

    void runFlute();
};
