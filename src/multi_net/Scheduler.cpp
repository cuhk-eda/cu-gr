#include "Scheduler.h"

vector<vector<int>> &Scheduler::schedule() {
    //
    vector<int> estimatedNumOfVertices(routers.size(), 0);
    for (int id = 0; id < routers.size(); ++id)
        for (auto &guide : routers[id].guides)
            estimatedNumOfVertices[id] += (guide[X].range() + 1) * (guide[Y].range() + 1);

    // init assigned table
    vector<bool> assigned(routers.size(), false);

    // sort by sizes
    vector<int> routerIds;
    for (int id = 0; id < routers.size(); ++id) routerIds.push_back(id);
    if (db::setting.multiNetScheduleSortAll) {
        std::sort(routerIds.begin(), routerIds.end(), [&](int lhs, int rhs) {
            return estimatedNumOfVertices[lhs] > estimatedNumOfVertices[rhs];
        });
    }

    if (db::setting.numThreads == 0) {
        // simple case
        for (int routerId : routerIds) batches.push_back({routerId});
    } else {
        // normal case
        int lastUnroute = 0;
        while (lastUnroute < routerIds.size()) {
            // create a new batch from a seed
            batches.emplace_back();
            initSet({});
            vector<int> &batch = batches.back();
            for (int i = lastUnroute; i < routerIds.size(); ++i) {
                int routerId = routerIds[i];

                if (!assigned[routerId] && !hasConflict(routerId)) {
                    batch.push_back(routerId);
                    assigned[routerId] = true;
                    updateSet(routerId);
                }
            }
            // find the next seed
            while (lastUnroute < routerIds.size() && assigned[routerIds[lastUnroute]]) {
                ++lastUnroute;
            }
        }

        // sort within batches by NumOfVertices
        if (db::setting.multiNetScheduleSort) {
            for (auto &batch : batches) {
                std::sort(batch.begin(), batch.end(), [&](int lhs, int rhs) {
                    return estimatedNumOfVertices[lhs] > estimatedNumOfVertices[rhs];
                });
            }
        }
    }

    if (db::setting.multiNetScheduleReverse) {
        reverse(batches.begin(), batches.end());
    }

    return batches;
}

void Scheduler::initSet(vector<int> jobIdxes) {
    rtrees = RTrees(database.getLayerNum());
    for (int jobIdx : jobIdxes) {
        updateSet(jobIdx);
    }
}

void Scheduler::updateSet(int jobIdx) {
    for (const auto &guide : routers[jobIdx].guides) {
        boostBox box(boostPoint(guide[X].low, guide[Y].low), boostPoint(guide[X].high, guide[Y].high));
        rtrees[guide.layerIdx].insert({box, jobIdx});
    }
}

bool Scheduler::hasConflict(int jobIdx) {
    for (const auto &guide : routers[jobIdx].guides) {
        boostBox box(boostPoint(guide[X].low, guide[Y].low), boostPoint(guide[X].high, guide[Y].high));

        std::vector<std::pair<boostBox, int>> results;
        rtrees[guide.layerIdx].query(bgi::intersects(box), std::back_inserter(results));

        for (const auto &result : results) {
            if (result.second != jobIdx) {
                return true;
            }
        }
    }
    return false;
}
