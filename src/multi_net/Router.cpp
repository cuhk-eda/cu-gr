#include "Router.h"
#include "flute/flute.h"
#include "Scheduler.h"
#include "single_net/InitRoute.h"

extern "C" {
void readLUT();
}

const MTStat& MTStat::operator+=(const MTStat& rhs) {
    auto dur = rhs.durations;
    std::sort(dur.begin(), dur.end());
    if (durations.size() < dur.size()) {
        durations.resize(dur.size(), 0.0);
    }
    for (int i = 0; i < dur.size(); ++i) {
        durations[i] += dur[i];
    }
    return *this;
}

ostream& operator<<(ostream& os, const MTStat mtStat) {
    double minDur = std::numeric_limits<double>::max(), maxDur = 0.0, avgDur = 0.0;
    for (double dur : mtStat.durations) {
        minDur = min(minDur, dur);
        maxDur = max(maxDur, dur);
        avgDur += dur;
    }
    avgDur /= mtStat.durations.size();
    os << "#threads=" << mtStat.durations.size() << " (dur: min=" << minDur << ", max=" << maxDur << ", avg=" << avgDur
       << ")";
    return os;
}

void Router::run() {
    allNetStatus.resize(database.nets.size(), db::RouteStatus::FAIL_UNPROCESSED);
    for (iter = 0; iter < db::setting.rrrIterLimit; iter++) {
        log() << std::endl;
        log() << "################################################################" << std::endl;
        log() << "Start RRR iteration " << iter << std::endl;
        log() << std::endl;
        db::routeStat.clear();
        guideGenStat.reset();

        vector<int> netsToRoute = getNetsToRoute();
        if (netsToRoute.empty()) {
            if (db::setting.multiNetVerbose >= +db::VerboseLevelT::MIDDLE) {
                log() << "No net is identified for this iteration of RRR." << std::endl;
                log() << std::endl;
            }
            break;
        }
        sortNets(netsToRoute);  // Note: only effective when doing mazeroute sequentially

        updateCost();
        grDatabase.statHistCost();

        if (iter > 0) {
            ripup(netsToRoute);
            congMap.init(cellWidth, cellHeight);
        }

        routeApprx(netsToRoute);

        log() << std::endl;
        log() << "Finish RRR iteration " << iter << std::endl;
        log() << "MEM: cur=" << utils::mem_use::get_current() << "MB, peak=" << utils::mem_use::get_peak() << "MB"
              << std::endl;
        if (db::setting.multiNetVerbose >= +db::VerboseLevelT::MIDDLE) db::routeStat.print();
    }
    // postprocessing
    for (auto& net : grDatabase.nets) {
        GuideGenerator guideGen(net);
        guideGen.genPatchGuides();
    }
    if (db::setting.multiNetVerbose >= +db::VerboseLevelT::MIDDLE) guideGenStat.print();

    log() << std::endl;
    log() << "################################################################" << std::endl;
    database.setUnitVioCost(1);  // set the cost back to without discount
    log() << "Finish all RRR iterations and PostRoute" << std::endl;
    log() << "MEM: cur=" << utils::mem_use::get_current() << "MB, peak=" << utils::mem_use::get_peak() << "MB"
          << std::endl;

    printStat();
}

Router::Router() {
    readLUT();  // read flute LUT
}

void Router::ripup(const vector<int>& netsToRoute) {
    for (auto id : netsToRoute) {
        grDatabase.removeNet(grDatabase.nets[id]);
        grDatabase.nets[id].gridTopo.clear();
        allNetStatus[id] = db::RouteStatus::FAIL_UNPROCESSED;
    }
}

void Router::updateCost() {
    if (iter > 0) {
        // apply in rrr stages
        grDatabase.addHistCost();
        grDatabase.fadeHistCost();

        grDatabase.setUnitViaMultiplier(
            max(100 / pow(5, iter - 1), 4.0));  // note: enlarge unit via cost to avoid extra use of vias
        grDatabase.setLogisticSlope(db::setting.initLogisticSlope * pow(2, iter));
    }

    if (db::setting.rrrIterLimit > 1) {
        double step = (1.0 - db::setting.rrrInitVioCostDiscount) / (db::setting.rrrIterLimit - 1);
        database.setUnitVioCost(db::setting.rrrInitVioCostDiscount + step * iter);
    }
}

vector<vector<int>> Router::getBatches(vector<SingleNetRouter>& routers, const vector<int>& netsToRoute) {
    vector<int> batch(netsToRoute.size());
    for (int i = 0; i < netsToRoute.size(); i++) batch[i] = i;

    runJobsMT(batch.size(), [&](int jobIdx) {
        auto& router = routers[batch[jobIdx]];

        const auto mergedPinAccessBoxes = grDatabase.nets[netsToRoute[jobIdx]].getMergedPinAccessBoxes(
            [](const gr::GrPoint& point) { return gr::PointOnLayer(point.layerIdx, point[X], point[Y]); });
        utils::IntervalT<int> xIntvl, yIntvl;
        for (auto& points : mergedPinAccessBoxes) {
            for (auto& point : points) {
                xIntvl.Update(point[X]);
                yIntvl.Update(point[Y]);
            }
        }
        router.guides.emplace_back(0, xIntvl, yIntvl);
    });

    Scheduler scheduler(routers);
    const vector<vector<int>>& batches = scheduler.schedule();

    if (db::setting.multiNetVerbose >= +db::VerboseLevelT::MIDDLE) {
        log() << "Finish multi-thread scheduling" << ((db::setting.numThreads == 0) ? " using simple mode" : "")
              << ". There will be " << batches.size() << " batches for " << netsToRoute.size() << " nets." << std::endl;
        log() << std::endl;
    }

    return batches;
}

void Router::routeApprx(const vector<int>& netsToRoute) {
    if (iter == 0) {
        fluteAllAndRoute(netsToRoute);
    } else {
        vector<SingleNetRouter> routers;
        routers.reserve(netsToRoute.size());
        for (auto id : netsToRoute) routers.emplace_back(grDatabase.nets[id]);

        vector<vector<int>> batches = getBatches(routers, netsToRoute);

        for (const vector<int>& batch : batches) {
            runJobsMT(batch.size(), [&](int jobIdx) {
                auto& router = routers[batch[jobIdx]];
                router.planMazeRoute(congMap);
            });

            for (auto jobIdx : batch) {
                auto& router = routers[jobIdx];
                router.mazeRoute();
                router.finish();

                int netIdx = netsToRoute[jobIdx];
                congMap.update(grDatabase.nets[netIdx]);
                allNetStatus[netIdx] = router.status;
            }
        }
    }
}

void Router::fluteAllAndRoute(const vector<int>& netsToRoute) {
    vector<SingleNetRouter> routers;
    vector<InitRoute> initRouters;
    routers.reserve(netsToRoute.size());
    initRouters.reserve(netsToRoute.size());
    for (auto id : netsToRoute) routers.emplace_back(grDatabase.nets[id]);
    for (auto id : netsToRoute) initRouters.emplace_back(grDatabase.nets[id]);

    grDatabase.init2DMaps(grDatabase);

    for (auto& router : initRouters)
        if (router.grNet.needToRoute()) router.plan_fluteOnly();
    printlog("finish planning");

    for (int i = 0; i < db::setting.edgeShiftingIter; i++) {
        grDatabase.edge_shifted = 0;
        for (auto& router : initRouters)
            if (router.grNet.needToRoute()) router.edge_shift2d(router.getRouteNodes());
        log() << "Total Number of edges in Iter " << i << " : " << grDatabase.tot_edge
              << ". Edge Shifted: " << grDatabase.edge_shifted << "("
              << 100.0 * grDatabase.edge_shifted / grDatabase.tot_edge << "%)" << std::endl;
    }
    for (auto& router : initRouters)
        if (router.grNet.needToRoute()) router.getRoutingOrder();
    printlog("finish edge shifting");

    for (int i = 0; i < routers.size(); i++) {
        auto& router = routers[i];
        router.initRoutePattern(initRouters[i]);
        router.finish();
        allNetStatus[netsToRoute[i]] = router.status;
    }

    printlog("finish pattern route");
}

void Router::sortNets(vector<int>& netsToRoute) {
    sort(netsToRoute.begin(), netsToRoute.end(), [&](int id1, int id2) {
        return grDatabase.nets[id1].boundingBox.hp() < grDatabase.nets[id2].boundingBox.hp();
    });
}

vector<int> Router::getNetsToRoute() {
    vector<int> netsToRoute;
    if (iter == 0) {
        for (auto& net : grDatabase.nets) netsToRoute.push_back(net.dbNet.idx);
    } else {
        for (auto& net : grDatabase.nets)
            if (grDatabase.hasVio(net)) netsToRoute.push_back(net.dbNet.idx);
    }

    return netsToRoute;
}

void Router::printStat() {
    log() << std::endl;
    log() << "----------------------------------------------------------------" << std::endl;
    db::routeStat.print();
    grDatabase.printAllUsageAndVio();
    log() << "----------------------------------------------------------------" << std::endl;
    log() << std::endl;
}
