#pragma once

#include "db/Database.h"
#include "GrGeoPrimitive.h"
#include "GCell.h"
#include "GridTopo.h"

namespace gr {
class GrNet {
public:
    db::Net& dbNet;

    vector<vector<GrPoint>> pinAccessBoxes;
    std::unordered_set<GrPoint> ovlpPoints;
    GrBoxOnLayer boundingBox;

    vector<std::shared_ptr<GrSteiner>> gridTopo;

    vector<GrBoxOnLayer> wireRouteGuides;
    vector<GrBoxOnLayer> viaRouteGuides;
    vector<GrBoxOnLayer> patchRouteGuides;

    GrNet(int i) : dbNet(database.nets[i]) {}
    void init(const GCellGrid& gcellGrid);

    unsigned numOfPins() const { return dbNet.numOfPins(); }
    const std::string& getName() const { return dbNet.getName(); }

    void postOrderVisitGridTopo(const std::function<void(std::shared_ptr<GrSteiner>)>& visit) const;
    void preOrderVisitGridTopo(const std::function<void(std::shared_ptr<GrSteiner>)>& visit) const;
    DBU getWirelength() const;

    vector<vector<PointOnLayer>> getMergedPinAccessBoxes(const std::function<PointOnLayer(GrPoint)>& pointHash) const;

    bool needToRoute() { return !isOnePin; }

private:
    void initPinAccessBoxes(const GCellGrid& gcellGrid);

    bool isOnePin = false;
};

class GrNetlist {
public:
    vector<GrNet> nets;

    void init(const GCellGrid& gcellGrid);
};
}  // namespace gr
