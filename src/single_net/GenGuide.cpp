#include "GenGuide.h"

GuideGeneratorStat guideGenStat;

void GuideGenerator::sliceGuides(vector<gr::GrBoxOnLayer> &guides, bool mergeAdj) {
    vector<vector<gr::GrBoxOnLayer>> tmpGuides(database.getLayerNum());  // route guides on different layers
    for (auto &guide : guides) {
        tmpGuides[guide.layerIdx].push_back(guide);
    }
    guides.clear();
    for (int layerIdx = 0; layerIdx < database.getLayerNum(); ++layerIdx) {
        gr::GrBoxOnLayer::sliceGrPolygons(tmpGuides[layerIdx], mergeAdj);
        for (auto &guide : tmpGuides[layerIdx]) guides.push_back(guide);
    }
}

void GuideGenerator::genConnGuides() {
    // keep connectivity
    for (auto &point : grNet.ovlpPoints) {
        grNet.viaRouteGuides.emplace_back(
            point.layerIdx, utils::IntervalT<int>(point[X], point[X]), utils::IntervalT<int>(point[Y], point[Y]));
        if (point.layerIdx + 1 < database.getLayerNum())
            grNet.viaRouteGuides.emplace_back(point.layerIdx + 1,
                                              utils::IntervalT<int>(point[X], point[X]),
                                              utils::IntervalT<int>(point[Y], point[Y]));
        if (point.layerIdx - 1 >= 0)
            grNet.viaRouteGuides.emplace_back(point.layerIdx - 1,
                                              utils::IntervalT<int>(point[X], point[X]),
                                              utils::IntervalT<int>(point[Y], point[Y]));
    }
}

void GuideGenerator::patchPinRegions() {
    double patchThresh = 2.0;
    // check if the region allows patching
    auto needPatch = [&](gr::GrPoint point) { return grDatabase.getCellResource(point) < patchThresh; };
    // get surrounding gcells of a point (normally 3 x 3, but will be different at boudary)
    auto getSurrounding = [&](gr::GrPoint point) {
        gr::GrBoxOnLayer box;
        box.layerIdx = point.layerIdx;
        box.x.low = max(point.x - 1, 0);
        box.y.low = max(point.y - 1, 0);
        box.x.high = min(point.x + 1, grDatabase.getNumGrPoint(X) - 1);
        box.y.high = min(point.y + 1, grDatabase.getNumGrPoint(Y) - 1);
        return box;
    };

    for (auto &pbxs : grNet.pinAccessBoxes) {
        for (auto &pbx : pbxs) {
            bool patched = false;
            // patch upper two layers
            if (pbx.layerIdx < database.getLayerNum() - 2) {
                guideGenStat.pinRegionPatchCand++;
                auto bxPlus1 = gr::GrPoint(pbx.layerIdx + 1, pbx.x, pbx.y);
                auto bxPlus2 = gr::GrPoint(pbx.layerIdx + 2, pbx.x, pbx.y);
                if (needPatch(bxPlus1) || needPatch(bxPlus2)) {
                    patched = true;
                    guideGenStat.pinRegionPatchNum++;
                    grNet.patchRouteGuides.emplace_back(getSurrounding(bxPlus1));
                    grNet.patchRouteGuides.emplace_back(getSurrounding(bxPlus2));
                } else {
                    grNet.patchRouteGuides.emplace_back(gr::GrBoxOnLayer(bxPlus1.layerIdx, {pbx.x}, {pbx.y}));
                    grNet.patchRouteGuides.emplace_back(gr::GrBoxOnLayer(bxPlus2.layerIdx, {pbx.x}, {pbx.y}));
                }
            }
            // patch lower two layers
            if (pbx.layerIdx > 1) {
                guideGenStat.pinRegionPatchCand++;
                auto bxMinus1 = gr::GrPoint(pbx.layerIdx - 1, pbx.x, pbx.y);
                auto bxMinus2 = gr::GrPoint(pbx.layerIdx - 2, pbx.x, pbx.y);
                if (needPatch(bxMinus1) || needPatch(bxMinus2)) {
                    patched = true;
                    guideGenStat.pinRegionPatchNum++;
                    grNet.patchRouteGuides.emplace_back(getSurrounding(bxMinus1));
                    grNet.patchRouteGuides.emplace_back(getSurrounding(bxMinus2));
                } else {
                    grNet.patchRouteGuides.emplace_back(gr::GrBoxOnLayer(bxMinus1.layerIdx, {pbx.x}, {pbx.y}));
                    grNet.patchRouteGuides.emplace_back(gr::GrBoxOnLayer(bxMinus2.layerIdx, {pbx.x}, {pbx.y}));
                }
            }
            // patch original layer
            if (patched) {
                grNet.patchRouteGuides.emplace_back(getSurrounding(pbx));
            } else {
                grNet.patchRouteGuides.emplace_back(gr::GrBoxOnLayer(pbx.layerIdx, {pbx.x}, {pbx.y}));
            }
        }
    }
}

void GuideGenerator::patchLongSegments() {
    // Note: assuming all guides are single width
    const int patchIntvl = 5;  // min patch interval
    const double patchThresh = 1.0;
    for (const auto &guide : grNet.wireRouteGuides) {
        int layerIdx = guide.layerIdx;
        auto dir = database.getLayerDir(layerIdx);
        int offset = grNet.dbNet.idx % patchIntvl;
        for (int cp = guide[1 - dir].low + offset; cp <= guide[1 - dir].high;) {
            int x = dir == X ? guide[dir].low : cp;
            int y = dir == X ? cp : guide[dir].low;
            if (grDatabase.getCellResource({layerIdx, x, y}) < patchThresh) {
                bool patched = false;
                for (int layer_delta = -1; layer_delta <= 1; layer_delta += 2) {
                    int layer = layerIdx + layer_delta;
                    if (layer < 1 || layer >= database.getLayerNum()) continue;
                    if (grDatabase.getCellResource({layer, x, y}) > patchThresh) {
                        grNet.patchRouteGuides.emplace_back(gr::GrBoxOnLayer(layer, {x}, {y}));
                        guideGenStat.longSegmentPatchNum++;
                        patched = true;
                    }
                }
                if (patched) {
                    cp += patchIntvl;
                } else {
                    cp++;
                }
            } else {
                cp++;
            }
        }
    }
}

void GuideGenerator::patchVioCells() {
    // Note: assuming all guides are single width
    const int queryWidth = 1;

    for (const auto &guide : grNet.wireRouteGuides) {
        int layerIdx = guide.layerIdx;
        auto dir = database.getLayerDir(layerIdx);
        int gridline = guide[dir].low;

        for (int cp = guide[1 - dir].low; cp <= guide[1 - dir].high; cp++) {
            int x = dir == X ? gridline : cp;
            int y = dir == X ? cp : gridline;

            double cellRsrc = grDatabase.getCellResource({layerIdx, x, y});

            if (cellRsrc <= 0) {
                guideGenStat.vioCellNum++;

                gr::GrBoxOnLayer patch(layerIdx, utils::IntervalT<int>(x), utils::IntervalT<int>(y));

                for (int g = gridline - queryWidth; g <= gridline + queryWidth; g++) {
                    if (g < 0 || g >= grDatabase.getNumGrPoint(dir)) continue;

                    double curcellRsrc = dir == X ? grDatabase.getCellResource({layerIdx, g, y})
                                                   : grDatabase.getCellResource({layerIdx, x, g});
                    if (curcellRsrc <= 0) continue;
                    cellRsrc += curcellRsrc;
                    patch[dir].Update(g);
                }

                if (cellRsrc > 0) {
                    const vector<int> layers = {layerIdx + 1, layerIdx - 1};
                    for (auto l : layers) {
                        if (l >= database.getLayerNum() || l <= 1) continue;

                        bool vioFree = true;
                        for (int x = patch.lx(); x <= patch.hx() && vioFree; x++)
                            for (int y = patch.ly(); y <= patch.hy() && vioFree; y++)
                                if (grDatabase.getCellResource({l, x, y}) <= 0) {
                                    vioFree = false;
                                }

                        if (vioFree) {
                            grNet.patchRouteGuides.push_back(patch);
                            grNet.patchRouteGuides.emplace_back(l, patch[X], patch[Y]);
                            guideGenStat.vioCellPatchNum++;
                            break;
                        }
                    }
                }
            }
        }
    }
}

void GuideGenerator::genPatchGuides() {
    grNet.patchRouteGuides.clear();
    patchPinRegions();
    patchLongSegments();
    patchVioCells();
}

void GuideGenerator::genTopoGuides() {
    grNet.wireRouteGuides.clear();
    grNet.viaRouteGuides.clear();

    genConnGuides();

    // Note: generate guides by topology
    grNet.postOrderVisitGridTopo([&](std::shared_ptr<gr::GrSteiner> node) {
        auto parent = node;
        for (auto child : parent->children) {
            if (parent->layerIdx == child->layerIdx) {
                std::shared_ptr<gr::GrSteiner> lower, upper;
                if ((*parent)[X] < (*child)[X] || (*parent)[Y] < (*child)[Y]) {
                    lower = parent;
                    upper = child;
                } else {
                    lower = child;
                    upper = parent;
                }
                grNet.wireRouteGuides.emplace_back(lower->layerIdx,
                                                   utils::IntervalT<int>((*lower)[X], (*upper)[X]),
                                                   utils::IntervalT<int>((*lower)[Y], (*upper)[Y]));
            } else {
                grNet.viaRouteGuides.emplace_back(parent->layerIdx,
                                                  utils::IntervalT<int>((*parent)[X], (*parent)[X]),
                                                  utils::IntervalT<int>((*parent)[Y], (*parent)[Y]));
                grNet.viaRouteGuides.emplace_back(child->layerIdx,
                                                  utils::IntervalT<int>((*child)[X], (*child)[X]),
                                                  utils::IntervalT<int>((*child)[Y], (*child)[Y]));
            }
        }
    });

    sliceGuides(grNet.wireRouteGuides);
    sliceGuides(grNet.viaRouteGuides);
}
