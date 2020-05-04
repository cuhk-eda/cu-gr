#pragma once

#include "GCell.h"
#include "GrGeoPrimitive.h"
#include "GrNet.h"

namespace gr {
class GrRouteGrid : public GCellGrid {
public:
    using UsageT = double;
    using UsageMapT = vector<vector<vector<UsageT>>>;
    int edge_shifted =0;
    int tot_edge = 0;

    void init();
    void clear();

    void useNet(const GrNet& net);
    void removeNet(GrNet& net);

    db::CostT getViaCost(const GrPoint& via) const;
    db::CostT getStackViaCost(const GrPoint& via, int height) const;
    db::CostT getWireCost(const GrEdge& edge) const;
    db::CostT getHistCost(int layerIdx, int gridline, int cp) const;

    double getCellResource(const GrPoint& point) const;  // Note: simplified version

    bool hasVio(const GrNet& net, bool hasCommit = true) const;
    bool hasVio(const GrEdge& edge, bool hasCommit = true) const;
    bool hasVio(const GrPoint& via, bool hasCommit = true) const;

    void print() const;
    void printAllUsageAndVio() const;

    double getWirelength() const;

    void setViaCapDiscount(double discount = 1);
    void setWireCapDiscount(double discount = 1);
    void setUnitViaMultiplier(double multiplier = 1);
    void setLogisticSlope(double slope = 1);

    double getLogisticSlope() const { return logisticSlope; }

    // for ripup and reroute
    void addHistCost();
    void fadeHistCost();
    void statHistCost() const;

    friend class GrRouteGrid2D;

private:
    // only pref-dir is modeled
    UsageMapT routedWireMap;  // model cross-cell routing
    UsageMapT routedViaMap;
    vector<vector<vector<std::pair<int, DBU>>>> fixedMetalMap;  // layer, x, y, (# blocked tracks, avg blocked length)
    vector<vector<vector<double>>> histWireUsageMap;

    db::CostT getWireDistCost(const GrEdge& edge) const;
    db::CostT getWireShortCost(const GrEdge& edge) const;
    db::CostT getViaShortCost(const GrPoint& via) const;

    double viaCapDiscount = 1;
    double wireCapDiscount = 1;
    double unitViaMultiplier = 1;
    double logisticSlope = 1;

    std::pair<double, double> printAllUsage() const;
    double printAllVio() const;

    double getAllWireUsage(const vector<double>& buckets,
                           vector<int>& wireUsageGrid,
                           vector<DBU>& wireUsageLength) const;
    void getAllInCellUsage(const vector<double>& buckets, vector<int>& viaUsage) const;
    double getTotViaNum() const;

    void markFixedMetals();
    void markFixed(int layerIdx, int gridline, int cp, int num_track, DBU avg_length);

    double getNumVio(const GrNet& net, bool hasCommit) const;
    double getNumVio(const GrEdge& edge, double selfUsage) const;
    double getNumVio(const GrPoint& via, double selfUsage) const;

    double getFixedUsage(int layerIdx, int gridline, int cp) const;
    double getFixedUsage(const GrEdge& edge) const;
    double getWireUsage(int layerIdx, int gridline, int cp) const;
    double getWireUsage(const GrEdge& edge) const;

    double getViaUsage(int layerIdx, int x, int y) const;
    double getViaUsage(const GrPoint& via) const;
    double getCellResource(int layerIdx, int x, int y) const;

    double getWireCapacity(const GrEdge& edge) const;
    double getInCellArea(const GrPoint& point) const;

    double getInCellUsedArea(const GrPoint& point) const;
    double getFixedUsedArea(const GrEdge& edge) const;  // fixed used area of the area of an edge
    DBU getFixedLength(const GrEdge& edge) const;
    double getInCellViaNum(const GrPoint& point) const;
    // double getUnitViaArea(const GrPoint& point, int side) const;

    void removeWire(const GrBoxOnLayer& box);
    void removeVia(const GrBoxOnLayer& box);

    void useWire(const GrBoxOnLayer& box);
    void useWire(int layerIdx, int gridline, int cp, double usage = 1);
    void useVia(const GrBoxOnLayer& box);
    void useVia(int layerIdx, int x, int y, double usage = 1);
    void useHistWire(int layerIdx, int gridline, int cp, double usage);
};

class GrRouteGrid2D {
public:
    using UsageT = double;
    using UsageMapT = vector<vector<vector<UsageT>>>;

    void init2DMaps(const GrRouteGrid& routeGrid);
    void useWire2D(int dir, int gridline, int cp, double usage = 1);
    void removeUsage2D(int dir, int gridline, int cp, double usage = 1);
    double getCost2D(int dir, int gridline, int cp) const;

private:
    UsageMapT wireUsageMap2D;   // 2d, first dim show X,Y
    UsageMapT fixedMetalMap2D;  // 2d, first dim show X,Y
    UsageMapT capacityMap2D;    // 2d, first dim show X,Y
};

}  // namespace gr
