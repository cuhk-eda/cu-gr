#include "Database.h"
#include "rsyn/io/parser/lef_def/DEFControlParser.h"

db::Database database;

namespace db {

void Database::init() {
    log() << std::endl;
    log() << "################################################################" << std::endl;
    log() << "Start initializing database" << std::endl;
    log() << std::endl;

    rsynService.init();

    auto dieBound = rsynService.physicalDesign.getPhysicalDie().getBounds();
    dieRegion = getBoxFromRsynBounds(dieBound);
    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        log() << "Die region (in DBU): " << dieRegion << std::endl;
        log() << std::endl;
    }

    RouteGrid::init();

    NetList::init(rsynService);

    markPinAndObsOccupancy();

    initMTSafeMargin();

    log() << "Finish initializing database" << std::endl;
    log() << "MEM: cur=" << utils::mem_use::get_current() << "MB, peak=" << utils::mem_use::get_peak() << "MB"
          << std::endl;
    log() << std::endl;
}

void Database::markPinAndObsOccupancy() {
    if (db::setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        log() << "Mark pin & obs occupancy on RouteGrid ..." << std::endl;
    }
    vector<std::pair<BoxOnLayer, int>> fixedMetalVec;

    // STEP 1: get fixed objects
    // Mark pins associated with nets
    for (const auto& net : nets) {
        for (const auto& accessBoxes : net.pinAccessBoxes) {
            for (const auto& box : accessBoxes) {
                fixedMetalVec.emplace_back(box, net.idx);
            }
        }
    }
    // Mark dangling pins
    // minor TODO: port?
    const Rsyn::Session session;
    const Rsyn::PhysicalDesign& physicalDesign =
        static_cast<Rsyn::PhysicalService*>(session.getService("rsyn.physical"))->getPhysicalDesign();
    const DBU libDBU = physicalDesign.getDatabaseUnits(Rsyn::LIBRARY_DBU);
    unsigned numUnusedPins = 0;
    unsigned numObs = 0;
    unsigned numSNetObs = 0;
    for (Rsyn::Instance instance : rsynService.module.allInstances()) {
        if (instance.getType() != Rsyn::CELL) continue;
        // phCell
        Rsyn::Cell cell = instance.asCell();
        Rsyn::PhysicalCell phCell = rsynService.physicalDesign.getPhysicalCell(cell);
        Rsyn::PhysicalLibraryCell phLibCell = rsynService.physicalDesign.getPhysicalLibraryCell(cell);
        const DBUxy origin(static_cast<DBU>(std::round(phLibCell.getMacro()->originX() * libDBU)),
                           static_cast<DBU>(std::round(phLibCell.getMacro()->originY() * libDBU)));
        // libPin
        for (Rsyn::Pin pin : instance.allPins(false)) {
            if (!pin.getNet()) {  // no associated net
                Rsyn::PhysicalLibraryPin phLibPin = rsynService.physicalDesign.getPhysicalLibraryPin(pin);
                vector<BoxOnLayer> accessBoxes;
                Net::getPinAccessBoxes(phLibPin, phCell, accessBoxes, origin);
                for (const auto& box : accessBoxes) {
                    fixedMetalVec.emplace_back(box, OBS_NET_IDX);
                }
                ++numUnusedPins;
            }
        }
        // libObs
        DBUxy displacement = phCell.getPosition() + origin;
        auto transform = phCell.getTransform();
        for (const Rsyn::PhysicalObstacle& phObs : phLibCell.allObstacles()) {
            if (phObs.getLayer().getType() != Rsyn::PhysicalLayerType::ROUTING) continue;
            const int layerIdx = phObs.getLayer().getRelativeIndex();
            for (auto bounds : phObs.allBounds()) {
                bounds.translate(displacement);
                bounds = transform.apply(bounds);
                const BoxOnLayer box(layerIdx, getBoxFromRsynBounds(bounds));
                fixedMetalVec.emplace_back(box, OBS_NET_IDX);
                ++numObs;
            }
        }
    }
    // Mark special nets
    for (Rsyn::PhysicalSpecialNet specialNet : rsynService.physicalDesign.allPhysicalSpecialNets()) {
        for (const DefWireDscp& wire : specialNet.getNet().clsWires) {
            for (const DefWireSegmentDscp& segment : wire.clsWireSegments) {
                int layerIdx =
                    rsynService.physicalDesign.getPhysicalLayerByName(segment.clsLayerName).getRelativeIndex();
                const DBU width = segment.clsRoutedWidth;
                DBUxy pos;
                DBU ext = 0;
                for (unsigned i = 0; i != segment.clsRoutingPoints.size(); ++i) {
                    const DefRoutingPointDscp& pt = segment.clsRoutingPoints[i];
                    const DBUxy& nextPos = pt.clsPos;
                    const DBU nextExt = pt.clsHasExtension ? pt.clsExtension : 0;
                    if (i >= 1) {
                        for (unsigned dim = 0; dim != 2; ++dim) {
                            if (pos[dim] == nextPos[dim]) continue;
                            const DBU l = pos[dim] < nextPos[dim] ? pos[dim] - ext : nextPos[dim] - nextExt;
                            const DBU h = pos[dim] < nextPos[dim] ? nextPos[dim] + nextExt : pos[dim] + ext;
                            BoxOnLayer box(layerIdx);
                            box[dim].Set(l, h);
                            box[1 - dim].Set(pos[1 - dim] - width / 2, pos[1 - dim] + width / 2);
                            fixedMetalVec.emplace_back(box, OBS_NET_IDX);
                            ++numSNetObs;
                            break;
                        }
                    }
                    pos = nextPos;
                    ext = nextExt;
                    if (!pt.clsHasVia) continue;
                    const Rsyn::PhysicalVia& via = rsynService.physicalDesign.getPhysicalViaByName(pt.clsViaName);
                    const int botLayerIdx = via.getBottomLayer().getRelativeIndex();
                    for (const Rsyn::PhysicalViaGeometry& geo : via.allBottomGeometries()) {
                        Bounds bounds = geo.getBounds();
                        bounds.translate(pos);
                        const BoxOnLayer box(botLayerIdx, getBoxFromRsynBounds(bounds));
                        fixedMetalVec.emplace_back(box, OBS_NET_IDX);
                        ++numSNetObs;
                    }
                    const int topLayerIdx = via.getTopLayer().getRelativeIndex();
                    for (const Rsyn::PhysicalViaGeometry& geo : via.allTopGeometries()) {
                        Bounds bounds = geo.getBounds();
                        bounds.translate(pos);
                        const BoxOnLayer box(topLayerIdx, getBoxFromRsynBounds(bounds));
                        fixedMetalVec.emplace_back(box, OBS_NET_IDX);
                        ++numSNetObs;
                    }
                    if (via.hasViaRule()) {
                        const utils::PointT<int> numRowCol =
                            via.hasRowCol() ? utils::PointT<int>(via.getNumCols(), via.getNumRows())
                                            : utils::PointT<int>(1, 1);
                        BoxOnLayer botBox(botLayerIdx);
                        BoxOnLayer topBox(topLayerIdx);
                        for (unsigned dimIdx = 0; dimIdx != 2; ++dimIdx) {
                            const Dimension dim = static_cast<Dimension>(dimIdx);
                            const DBU origin = via.hasOrigin() ? pos[dim] + via.getOrigin(dim) : pos[dim];
                            const DBU botOff =
                                via.hasOffset() ? origin + via.getOffset(Rsyn::BOTTOM_VIA_LEVEL, dim) : origin;
                            const DBU topOff =
                                via.hasOffset() ? origin + via.getOffset(Rsyn::TOP_VIA_LEVEL, dim) : origin;
                            const DBU length =
                                (via.getCutSize(dim) * numRowCol[dim] + via.getSpacing(dim) * (numRowCol[dim] - 1)) / 2;
                            const DBU botEnc = length + via.getEnclosure(Rsyn::BOTTOM_VIA_LEVEL, dim);
                            const DBU topEnc = length + via.getEnclosure(Rsyn::TOP_VIA_LEVEL, dim);
                            botBox[dim].Set(botOff - botEnc, botOff + botEnc);
                            topBox[dim].Set(topOff - topEnc, topOff + topEnc);
                        }
                        fixedMetalVec.emplace_back(botBox, OBS_NET_IDX);
                        fixedMetalVec.emplace_back(topBox, OBS_NET_IDX);
                        numSNetObs += 2;
                    }
                    if (layerIdx == botLayerIdx)
                        layerIdx = topLayerIdx;
                    else if (layerIdx == topLayerIdx)
                        layerIdx = botLayerIdx;
                    else {
                        log() << "Error: Special net " << specialNet.getNet().clsName << " via " << pt.clsViaName
                              << " on wrong layer " << layerIdx << std::endl;
                        break;
                    }
                }
            }
        }
    }
    // Stat
    vector<int> layerNumFixedObjects(getLayerNum(), 0);
    for (const auto& fixedMetal : fixedMetalVec) {
        layerNumFixedObjects[fixedMetal.first.layerIdx]++;
    }
    // Print
    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        log() << "The number of unused pins is " << numUnusedPins << std::endl;
        log() << "The number of OBS is " << numObs << std::endl;
        log() << "The number of special net OBS is " << numSNetObs << std::endl;
        log() << "The number of fixed objects on each layers:" << std::endl;
        for (unsigned i = 0; i < getLayerNum(); i++) {
            if (layerNumFixedObjects[i] > 0) log() << getLayer(i).name << ": " << layerNumFixedObjects[i] << std::endl;
        }
    }
    log() << std::endl;

    // STEP 2: mark
    if (setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
        printlog("mark fixed metal rtrees...");
    }

    markFixedMetalBatch(fixedMetalVec, 0, fixedMetalVec.size());
}

void Database::initMTSafeMargin() {
    for (auto& layer : layers) {
        layer.mtSafeMargin = max({layer.minAreaMargin, layer.confLutMargin, layer.fixedMetalQueryMargin});
        if (db::setting.dbVerbose >= +db::VerboseLevelT::MIDDLE) {
            printlog(layer.name,
                     "mtSafeMargin = max {",
                     layer.minAreaMargin,
                     layer.confLutMargin,
                     layer.fixedMetalQueryMargin,
                     "} =",
                     layer.mtSafeMargin);
        }
    }
}

void Database::getGridPinAccessBoxes(const Net& net, vector<vector<db::GridBoxOnLayer>>& gridPinAccessBoxes) const {
    gridPinAccessBoxes.resize(net.numOfPins());
    for (unsigned pinIdx = 0; pinIdx != net.numOfPins(); ++pinIdx) {
        vector<vector<db::GridBoxOnLayer>> pins(getLayerNum());
        for (const db::BoxOnLayer& pinAccessBox : net.pinAccessBoxes[pinIdx]) {
            int dir = getLayerDir(pinAccessBox.layerIdx);
            DBU pitch = getLayer(pinAccessBox.layerIdx).pitch;
            // pinForbidRegion
            auto pinForbidRegion = getMetalRectForbidRegion(pinAccessBox, AggrParaRunSpace::DEFAULT);
            const db::GridBoxOnLayer& gridPinForbidRegion = rangeSearch(pinForbidRegion);
            if (isValid(gridPinForbidRegion)) {
                pins[pinAccessBox.layerIdx].push_back(gridPinForbidRegion);
            }
            // One-pitch extension
            auto pinExtension = pinAccessBox;
            for (int d = 0; d < 2; ++d) {
                pinExtension[d].low -= pitch;
                pinExtension[d].high += pitch;
            }
            const db::GridBoxOnLayer& gridPinExtension = rangeSearch(pinExtension);
            for (int trackIdx = gridPinExtension.trackRange.low; trackIdx <= gridPinExtension.trackRange.high;
                 ++trackIdx) {
                for (int cpIdx = gridPinExtension.crossPointRange.low; cpIdx <= gridPinExtension.crossPointRange.high;
                     ++cpIdx) {
                    db::GridPoint pt(pinAccessBox.layerIdx, trackIdx, cpIdx);
                    if (!gridPinForbidRegion.includePoint(pt) && Dist(pinAccessBox, getLoc(pt)) <= pitch) {
                        pins[pinAccessBox.layerIdx].emplace_back(pinAccessBox.layerIdx,
                                                                 utils::IntervalT<int>{trackIdx, trackIdx},
                                                                 utils::IntervalT<int>{cpIdx, cpIdx});
                    }
                }
            }
        }

        // assign a relatively far grid access box if none (rarely happen)
        unsigned numBoxes = 0;
        for (const vector<db::GridBoxOnLayer>& pin : pins) {
            numBoxes += pin.size();
        }
        if (!numBoxes) {
            for (const db::BoxOnLayer& pinAccessBox : net.pinAccessBoxes[pinIdx]) {
                db::GridBoxOnLayer gridBox = rangeSearch(pinAccessBox);
                if (gridBox.trackRange.low > gridBox.trackRange.high) {
                    if (gridBox.trackRange.low == 0) {
                        gridBox.trackRange.high = 0;
                    } else {
                        gridBox.trackRange.low = gridBox.trackRange.high;
                    }
                }
                if (gridBox.crossPointRange.low > gridBox.crossPointRange.high) {
                    if (gridBox.crossPointRange.low == 0) {
                        gridBox.crossPointRange.high = 0;
                    } else {
                        gridBox.crossPointRange.low = gridBox.crossPointRange.high;
                    }
                }
                pins[pinAccessBox.layerIdx].push_back(gridBox);
            }
        }

        // slice
        gridPinAccessBoxes[pinIdx].clear();
        for (vector<db::GridBoxOnLayer>& pin : pins) {
            if (!pin.empty()) {
                db::GridBoxOnLayer::sliceGridPolygons(pin);
                for (const db::GridBoxOnLayer& box : pin) {
                    if (isValid(box)) {
                        gridPinAccessBoxes[pinIdx].push_back(box);
                    }
                }
            }
        }
        if (gridPinAccessBoxes[pinIdx].empty()) {
            log() << "Error: Net " << net.getName() << " Pin " << pinIdx << " has empty grid pin access boxes\n";
        }
    }
}

}  // namespace db

MTStat runJobsMT(int numJobs, const std::function<void(int)>& handle) {
    int numThreads = min(numJobs, db::setting.numThreads);
    MTStat mtStat(max(1, db::setting.numThreads));
    if (numThreads <= 1) {
        utils::timer threadTimer;
        for (int i = 0; i < numJobs; ++i) {
            handle(i);
        }
        mtStat.durations[0] = threadTimer.elapsed();
    } else {
        int globalJobIdx = 0;
        std::mutex mtx;
        utils::timer threadTimer;
        auto thread_func = [&](int threadIdx) {
            int jobIdx;
            while (true) {
                mtx.lock();
                jobIdx = globalJobIdx++;
                mtx.unlock();
                if (jobIdx >= numJobs) {
                    mtStat.durations[threadIdx] = threadTimer.elapsed();
                    break;
                }
                handle(jobIdx);
            }
        };

        std::thread threads[numThreads];
        for (int i = 0; i < numThreads; i++) {
            threads[i] = std::thread(thread_func, i);
        }
        for (int i = 0; i < numThreads; i++) {
            threads[i].join();
        }
    }
    return mtStat;
}
