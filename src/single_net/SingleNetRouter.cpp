#include "SingleNetRouter.h"
#include "InitRoute.h"
#include "MazeRoute.h"

SingleNetRouter::SingleNetRouter(gr::GrNet& grDatabaseNet)
    : grNet(grDatabaseNet), guideGen(grNet), status(db::RouteStatus::SUCC_NORMAL) {
    grDatabase.removeNet(grNet);
    grNet.gridTopo.clear();
}

void SingleNetRouter::finish() {
    guideGen.genTopoGuides();
    grDatabase.useNet(grNet);
}

void SingleNetRouter::mazeRoute() {
    if (!grNet.needToRoute()) {
        status = db::RouteStatus::SUCC_ONE_PIN;
    } else {
        MazeRoute mazeRouter(grNet);
        mazeRouter.constructGridGraph(guides);
        status = mazeRouter.run();
    }

    db::routeStat.increment(db::RouteStage::MAZE, status);
}

void SingleNetRouter::initRoutePattern(InitRoute& initRouter) {
    if (!grNet.needToRoute()) {
        status = db::RouteStatus::SUCC_ONE_PIN;
    } else {
        initRouter.patternRoute();
        initRouter.buildTopo();
        status = db::RouteStatus::SUCC_NORMAL;
    }
    db::routeStat.increment(db::RouteStage::INIT, status);
}

void SingleNetRouter::planMazeRoute(const CongestionMap& congMap) {
    // run mazeroute on coarse grid
    const int cellWidth = congMap.getCellWidth();
    const int cellHeight = congMap.getCellHeight();

    gr::GrNet tmpNet = grNet;
    MazeRoute mazeRouter(tmpNet);
    mazeRouter.constructGridGraph(congMap);
    status = mazeRouter.run();

    // generate guides
    auto getLower = [&](int coor, Dimension dir) {
        if (dir == X)
            return coor * cellWidth;
        else
            return coor * cellHeight;
    };
    auto getUpper = [&](int coor, Dimension dir) {
        if (dir == X)
            return min((coor + 1) * cellWidth, grDatabase.getNumGrPoint(X)) - 1;
        else
            return min((coor + 1) * cellHeight, grDatabase.getNumGrPoint(Y)) - 1;
    };

    guides.clear();
    tmpNet.postOrderVisitGridTopo([&](std::shared_ptr<gr::GrSteiner> node) {
        auto parent = node;
        for (auto child : parent->children) {
            // if (tmpNet.getName() == "net6700") printlog(*parent, *child);
            if (parent->layerIdx == child->layerIdx) {
                std::shared_ptr<gr::GrSteiner> lower, upper;
                if ((*parent)[X] < (*child)[X] || (*parent)[Y] < (*child)[Y]) {
                    lower = parent;
                    upper = child;
                } else {
                    lower = child;
                    upper = parent;
                }
                guides.emplace_back(lower->layerIdx,
                                    utils::IntervalT<int>(getLower((*lower)[X], X), getUpper((*upper)[X], X)),
                                    utils::IntervalT<int>(getLower((*lower)[Y], Y), getUpper((*upper)[Y], Y)));
            } else {
                guides.emplace_back(parent->layerIdx,
                                    utils::IntervalT<int>(getLower((*parent)[X], X), getUpper((*parent)[X], X)),
                                    utils::IntervalT<int>(getLower((*parent)[Y], Y), getUpper((*parent)[Y], Y)));
                guides.emplace_back(child->layerIdx,
                                    utils::IntervalT<int>(getLower((*child)[X], X), getUpper((*child)[X], X)),
                                    utils::IntervalT<int>(getLower((*child)[Y], Y), getUpper((*child)[Y], Y)));
            }
        }
    });

    // maintain connectivity
    auto mergedPinAccessBoxes = grNet.getMergedPinAccessBoxes([&](const gr::GrPoint& point) {
        return gr::PointOnLayer(point.layerIdx, point[X] / cellWidth, point[Y] / cellHeight);
    });
    const int neighLayers = 2;
    for (auto& points : mergedPinAccessBoxes) {
        for (auto& point : points) {
            for (int l = point.layerIdx - neighLayers; l <= point.layerIdx + neighLayers; l++) {
                if (l < database.getLayerNum() && l >= 0)
                    guides.emplace_back(l,
                                        utils::IntervalT<int>(getLower(point[X], X), getUpper(point[X], X)),
                                        utils::IntervalT<int>(getLower(point[Y], Y), getUpper(point[Y], Y)));
            }
        }
    }
}
