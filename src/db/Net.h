#pragma once

#include "RsynService.h"
#include "GeoPrimitive.h"

namespace db {

class NetBase {
public:
    ~NetBase();

    int idx;
    Rsyn::Net rsynNet;
    const std::string& getName() const { return rsynNet.getName(); }

    // pins
    vector<Rsyn::Pin> rsynPins;
    vector<vector<BoxOnLayer>> pinAccessBoxes;  // (pinIdx, accessBoxIdx) -> BoxOnLayer
    unsigned numOfPins() const noexcept { return pinAccessBoxes.size(); }
    BoxOnLayer getMaxAccessBox(int pinIdx) const;

    // route guides
    vector<BoxOnLayer> routeGuides;
    vector<GridBoxOnLayer> gridRouteGuides;

    void print(ostream& os = std::cout) const;
};

class Net : public NetBase {
public:
    Net(int i, Rsyn::Net net, RsynService& rsynService);

    // more route guide information
    vector<int> routeGuideVios;
    RTrees routeGuideRTrees;

    // for initialization
    void initPinAccessBoxes(Rsyn::Pin rsynPin,
                            RsynService& rsynService,
                            vector<BoxOnLayer>& accessBoxes,
                            const DBU libDBU);
    static void getPinAccessBoxes(Rsyn::PhysicalPort phPort, vector<BoxOnLayer>& accessBoxes);
    static void getPinAccessBoxes(Rsyn::PhysicalLibraryPin phLibPin,
                                  Rsyn::PhysicalCell phCell,
                                  vector<BoxOnLayer>& accessBoxes,
                                  const DBUxy& origin);

    void clearPostRouteResult();
    void clearResult();
};

class NetList {
public:
    vector<Net> nets;

    void init(RsynService& rsynService);
};

}  // namespace db