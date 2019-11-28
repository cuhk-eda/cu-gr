#include "RouteGrid.h"

namespace db {

void RouteGrid::init() {
    if (db::setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        log() << "Init RouteGrid ..." << std::endl;
    }
    LayerList::init();
    // Fixed metal
    fixedMetals.resize(layers.size());

    DBU m2Pitch = layers[1].pitch;
    unitWireCostRaw = db::Setting::weightWirelength / m2Pitch;
    unitViaCostRaw = db::Setting::weightViaNum;
    unitViaCost = unitViaCostRaw / unitWireCostRaw;

    unitShortVioCostRaw = db::Setting::weightShortArea;

    // note: a short of M2 track segments will be charged by
    // (shortLength * unitWireCostRaw) * unitShortVioCost
    // which should be unitShortVioCostRaw * (shortLength / m2Pitch) * (layers[i].width / m2Pitch)
    // Therefore, unitShortVioCost = unitShortVioCostRaw * layers[i].width / m2Pitch / m2Pitch / unitWireCostRaw
    unitShortVioCost.resize(layers.size());
    for (int i = 0; i < layers.size(); ++i) {
        unitShortVioCost[i] = unitShortVioCostRaw * layers[i].width / m2Pitch / m2Pitch / unitWireCostRaw;
    }

    unitShortVioCostDiscounted.resize(unitShortVioCost.size());
    setUnitVioCost();

    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        // LayerList::print();
        log() << "ROUTE GRID COST" << std::endl;
        log() << "unitWireCostRaw (score / DBU) = " << unitWireCostRaw << std::endl;
        log() << "unitViaCostRaw (score for each via) = " << unitViaCostRaw << std::endl;
        log() << "unitShortVioCostRaw (score for each area of m2Pitch * m2Pitch) = " << unitShortVioCostRaw
              << std::endl;
        log() << "After normalization by unitWireCostRaw: " << std::endl;
        log() << "unitWireCost (in DBU) = 1" << std::endl;
        log() << "unitViaCost (in DBU) = " << unitViaCost << std::endl;
        log() << "unitShortVioCost (coeff for \"normal\" short) = " << unitShortVioCost << std::endl;
        log() << std::endl;
    }
}

void RouteGrid::clear() {
    // Fixed metal
    fixedMetals.clear();
}

void RouteGrid::setUnitVioCost(double discount) {
    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        printlog("Set unit vio cost with discount of", discount);
    }
    for (int i = 0; i < unitShortVioCost.size(); ++i) {
        unitShortVioCostDiscounted[i] = unitShortVioCost[i] * discount;
    }
}

int RouteGrid::getFixedMetalVio(const BoxOnLayer& box, int netIdx) const {
    auto regions = getAccurateMetalRectForbidRegions(box);
    utils::BoxT<DBU> queryBox = box;
    queryBox.x.low -= layers[box.layerIdx].fixedMetalQueryMargin;
    queryBox.x.high += layers[box.layerIdx].fixedMetalQueryMargin;
    queryBox.y.low -= layers[box.layerIdx].fixedMetalQueryMargin;
    queryBox.y.high += layers[box.layerIdx].fixedMetalQueryMargin;

    for (const auto& region : regions) {
        queryBox = queryBox.UnionWith(region);
    }

    boostBox rtreeQueryBox(boostPoint(queryBox.x.low, queryBox.y.low), boostPoint(queryBox.x.high, queryBox.y.high));
    vector<std::pair<boostBox, int>> queryResults;
    fixedMetals[box.layerIdx].query(bgi::intersects(rtreeQueryBox), std::back_inserter(queryResults));
    vector<std::pair<utils::BoxT<DBU>, int>> neighMetals;
    for (const auto& queryResult : queryResults) {
        if (queryResult.second != netIdx) {
            const auto& b = queryResult.first;
            neighMetals.emplace_back(utils::BoxT<DBU>(bg::get<bg::min_corner, 0>(b),
                                                      bg::get<bg::min_corner, 1>(b),
                                                      bg::get<bg::max_corner, 0>(b),
                                                      bg::get<bg::max_corner, 1>(b)),
                                     queryResult.second);
        }
    }

    int numOvlp = 0;
    for (const auto& neighMetal : neighMetals) {
        // getOvlpFixedMetals
        for (auto forbidRegion : regions) {
            auto ovlp = forbidRegion.IntersectWith(neighMetal.first);
            if (ovlp.IsValid()) {
                numOvlp += (ovlp.area() > 0);
            }
        }

        // getOvlpFixedMetalForbidRegions
        auto forbidRegions = getAccurateMetalRectForbidRegions({box.layerIdx, neighMetal.first});
        for (auto forbidRegion : forbidRegions) {
            auto ovlp = forbidRegion.IntersectWith(box);
            if (ovlp.IsValid()) {
                numOvlp += (ovlp.area() > 0);
            }
        }

        // getOvlpC2CMetals
        if (!layers[box.layerIdx].isEolDominated(neighMetal.first)) {
            DBU space = layers[box.layerIdx].getParaRunSpace(neighMetal.first);
            numOvlp += (utils::L2Dist(box, neighMetal.first) < space);
        }
    }
    return numOvlp;
}

DBU RouteGrid::getOvlpFixedMetalArea(const BoxOnLayer& box, int netIdx) const {
    utils::BoxT<DBU> queryBox = box;

    boostBox rtreeQueryBox(boostPoint(queryBox.x.low, queryBox.y.low), boostPoint(queryBox.x.high, queryBox.y.high));
    vector<std::pair<boostBox, int>> queryResults;
    fixedMetals[box.layerIdx].query(bgi::intersects(rtreeQueryBox), std::back_inserter(queryResults));
    vector<std::pair<utils::BoxT<DBU>, int>> neighMetals;
    for (const auto& queryResult : queryResults) {
        if (queryResult.second != netIdx) {
            const auto& b = queryResult.first;
            neighMetals.emplace_back(utils::BoxT<DBU>(bg::get<bg::min_corner, 0>(b),
                                                      bg::get<bg::min_corner, 1>(b),
                                                      bg::get<bg::max_corner, 0>(b),
                                                      bg::get<bg::max_corner, 1>(b)),
                                     queryResult.second);
        }
    }

    DBU area = 0;
    for (const auto& neighMetal : neighMetals) {
        auto ovlp = queryBox.IntersectWith(neighMetal.first);
        if (ovlp.IsValid()) area += ovlp.area();
    }
    return area;
}

vector<std::pair<utils::BoxT<DBU>, int>> RouteGrid::getOvlpBoxes(const BoxOnLayer& box,
                                                                 int idx,
                                                                 const RTrees& rtrees) const {
    boostBox queryBox(boostPoint(box.x.low, box.y.low), boostPoint(box.x.high, box.y.high));
    vector<std::pair<boostBox, int>> queryResults;
    rtrees[box.layerIdx].query(bgi::intersects(queryBox), std::back_inserter(queryResults));
    vector<std::pair<utils::BoxT<DBU>, int>> results;
    for (const auto& queryResult : queryResults) {
        if (queryResult.second != idx) {
            const auto& b = queryResult.first;
            results.emplace_back(utils::BoxT<DBU>(bg::get<bg::min_corner, 0>(b),
                                                  bg::get<bg::min_corner, 1>(b),
                                                  bg::get<bg::max_corner, 0>(b),
                                                  bg::get<bg::max_corner, 1>(b)),
                                 queryResult.second);
        }
    }
    return results;
}

vector<std::pair<utils::BoxT<DBU>, int>> RouteGrid::getOvlpFixedMetals(const BoxOnLayer& box, int netIdx) const {
    return getOvlpBoxes(box, netIdx, fixedMetals);
}

void RouteGrid::markFixedMetalBatch(vector<std::pair<BoxOnLayer, int>>& fixedMetalVec, int beginIdx, int endIdx) {
    vector<vector<std::pair<boostBox, int>>> fixedMetalsRtreeItems;
    fixedMetalsRtreeItems.resize(layers.size());

    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        log() << "mark fixed metal batch ..." << std::endl;
    }
    const int initMem = utils::mem_use::get_current();

    vector<vector<int>> layerToObjIdx(getLayerNum());
    for (unsigned i = beginIdx; i < endIdx; i++) layerToObjIdx[fixedMetalVec[i].first.layerIdx].push_back(i);

    int curLayer = 0;
    std::mutex layer_mutex;

    auto thread_func = [&]() {
        while (true) {
            layer_mutex.lock();
            int l = curLayer++;
            layer_mutex.unlock();

            if (l >= getLayerNum()) return;

            for (auto idx : layerToObjIdx[l]) {
                // fixedMetals
                const BoxOnLayer& box = fixedMetalVec[idx].first;
                int netIdx = fixedMetalVec[idx].second;

                boostBox markBox(boostPoint(box.x.low, box.y.low), boostPoint(box.x.high, box.y.high));
                fixedMetalsRtreeItems[box.layerIdx].push_back({markBox, netIdx});

                DBU space = layers[box.layerIdx].getParaRunSpace(box);
                if (space > layers[box.layerIdx].fixedMetalQueryMargin) {
                    layers[box.layerIdx].fixedMetalQueryMargin = space;
                }
            }
        }
    };

    const int numThreads = max(1, db::setting.numThreads);
    std::thread threads[numThreads];
    for (int i = 0; i < numThreads; i++) threads[i] = std::thread(thread_func);
    for (int i = 0; i < numThreads; i++) threads[i].join();

    const int curMem = utils::mem_use::get_current();
    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        printflog("MEM(MB): init/cur=%d/%d, incr=%d\n", initMem, curMem, curMem - initMem);
        log() << std::endl;
    }

    if (!beginIdx) {
        for (int layerIdx = 0; layerIdx < layers.size(); layerIdx++) {
            RTree tRtree(fixedMetalsRtreeItems[layerIdx]);
            fixedMetals[layerIdx] = boost::move(tRtree);
        }
    } else {
        for (int layerIdx = 0; layerIdx < layers.size(); layerIdx++) {
            for (auto& item : fixedMetalsRtreeItems[layerIdx]) fixedMetals[layerIdx].insert(item);
        }
    }
}
}  // namespace db
