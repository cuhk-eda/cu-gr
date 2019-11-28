#pragma once
#include "db/Database.h"
#include "gr_db/GrDatabase.h"
#include "multi_net/CongestionMap.h"
#include "GenGuide.h"

class InitRoute;

class SingleNetRouter {
public:
    gr::GrNet& grNet;

    db::RouteStatus status;

    SingleNetRouter(gr::GrNet& grNet);

    void planMazeRoute(const CongestionMap& congMap);
    void mazeRoute();

    void initRoutePattern(InitRoute& initRouter);

    void finish();

    vector<gr::GrBoxOnLayer> guides;

private:
    GuideGenerator guideGen;
};
