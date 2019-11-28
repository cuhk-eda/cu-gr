#pragma once

#include "db/Database.h"
#include "gr_db/GrDatabase.h"

// create remaining gcell resource map (for congestion overview)
class CongestionMap {
public:
    double getCrsnEdgeCost(const gr::PointOnLayer& u, const gr::PointOnLayer& v) const;
    double getCrsnViaCost(const gr::PointOnLayer& via) const;

    int getCellWidth() const { return xCrsnScale; }
    int getCellHeight() const { return yCrsnScale; }
    gr::PointOnLayer grPointToPoint(const gr::GrPoint& point) const;
    double getRsrcUsage(int l, int x, int y) const { return rsrcMap[l][x][y]; };

    void init(int x_crsn_scale, int y_crsn_scale);
    void update(const gr::GrNet& net);

private:
    bool getCrsnFixed(const gr::PointOnLayer& u) const;

    utils::BoxT<int> getGrBox(const gr::PointOnLayer& u) const;  // get the gcell box in a coarsened cell

    void updateViaCostMap(const gr::PointOnLayer& point);
    void updateWireCostMap(const gr::PointOnLayer& point);
    double calcCrsnEdgeCost(const gr::PointOnLayer& u, const gr::PointOnLayer& v);
    double calcCrsnViaCost(const gr::PointOnLayer& via);

    int xCrsnScale;
    int yCrsnScale;
    int xNumCrsnCell;
    int yNumCrsnCell;

    vector<vector<vector<double>>> rsrcMap;  // note that this is not coarsened
    vector<vector<vector<double>>> crsnEdgeCostMap;
    vector<vector<vector<double>>> crsnViaCostMap;
};
