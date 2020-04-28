#include "CutLayer.h"
#include "Setting.h"

namespace db {

ViaType::ViaType(Rsyn::PhysicalVia rsynVia) {
    if (rsynVia.allCutGeometries().size() > 1) hasMultiCut = true;

    bot = getBoxFromRsynGeometries(rsynVia.allBottomGeometries());
    cut = getBoxFromRsynGeometries(rsynVia.allCutGeometries());
    top = getBoxFromRsynGeometries(rsynVia.allTopGeometries());
    name = rsynVia.getName();

    if (rsynVia.hasRowCol()) {
        const DBU xBotEnc = rsynVia.getEnclosure(Rsyn::BOTTOM_VIA_LEVEL, X);
        const DBU yBotEnc = rsynVia.getEnclosure(Rsyn::BOTTOM_VIA_LEVEL, Y);
        const DBU xTopEnc = rsynVia.getEnclosure(Rsyn::TOP_VIA_LEVEL, X);
        const DBU yTopEnc = rsynVia.getEnclosure(Rsyn::TOP_VIA_LEVEL, Y);
        const int numRows = rsynVia.getNumRows();
        const int numCols = rsynVia.getNumCols();
        const DBU xCut = (rsynVia.getCutSize(X) * numCols + rsynVia.getSpacing(X) * (numCols - 1) + 1) / 2;
        const DBU yCut = (rsynVia.getCutSize(Y) * numRows + rsynVia.getSpacing(Y) * (numRows - 1) + 1) / 2;
        bot = {-xCut - xBotEnc, -yCut - yBotEnc, xCut + xBotEnc, yCut + yBotEnc};
        cut = {-xCut, -yCut, xCut, yCut};
        top = {-xCut - xTopEnc, -yCut - yTopEnc, xCut + xTopEnc, yCut + yTopEnc};
    }

    if (!bot.IsStrictValid() || !cut.IsStrictValid() || !top.IsStrictValid()) {
        log() << "Warning in " << __func__ << ": For " << rsynVia.getName()
              << " , has non strict valid via layer bound... " << std::endl;
    }
}

std::tuple<DBU, DBU, DBU, DBU> ViaType::getDefaultScore(const Dimension botDim, const Dimension topDim) const {
    return std::tuple<DBU, DBU, DBU, DBU>(bot[botDim].range(),       // belowWidth
                                          top[topDim].range(),       // aboveWidth
                                          bot[1 - botDim].range(),   // belowWidth
                                          top[1 - topDim].range());  // aboveLength
}

utils::BoxT<DBU> ViaType::getShiftedBotMetal(const utils::PointT<DBU>& viaPos) const {
    utils::BoxT<DBU> metal = bot;
    metal.ShiftBy(viaPos);
    return metal;
}
utils::BoxT<DBU> ViaType::getShiftedTopMetal(const utils::PointT<DBU>& viaPos) const {
    utils::BoxT<DBU> metal = top;
    metal.ShiftBy(viaPos);
    return metal;
}

CutLayer::CutLayer(const Rsyn::PhysicalLayer& rsynLayer,
                   const vector<Rsyn::PhysicalVia>& rsynVias,
                   const Dimension botDim,
                   const Dimension topDim,
                   const DBU libDBU)
    : name(rsynLayer.getName()), idx(rsynLayer.getRelativeIndex()), width(rsynLayer.getWidth()) {
    //  Rsyn::PhysicalLayer (LEF)
    const lefiLayer* layer = rsynLayer.getLayer();
    if (layer->hasSpacingNumber() && layer->numSpacing()) {
        spacing = std::lround(layer->spacing(0) * libDBU);
    }

    if (layer->numProps()) {
        for (unsigned iProp = 0; static_cast<int>(iProp) < layer->numProps(); ++iProp) {
            if (!strcmp(layer->propName(iProp), "LEF58_SPACINGTABLE")) {
                std::istringstream iss(layer->propValue(iProp));
                std::string sBuf("");
                double space{0};
                while (iss) {
                    iss >> sBuf;
                    if (sBuf == ";") {
                        continue;
                    }

                    if (sBuf == "DEFAULT") {
                        iss >> space;
                        if (!spacing) {
                            spacing = std::lround(space * libDBU);
                        } else if (std::lround(space * libDBU) != spacing) {
                            log() << "Warning in " << __func__ << ": For " << name << ", mismatched defaultSpace & spacingTable... " << std::endl;
                        }
                    }
                }
            } else {
                log() << "Warning in " << __func__ << ": For " << name << ", unknown prop: " << layer->propName(iProp) << "...\n";
            }
        }
    }

    if (!spacing) {
        log() << "Error in " << __func__ << ": For " << name << " CutLayer::init, rsynSpacingRule is empty, init all rules with default 0... " << std::endl;
    }
    delete rsynLayer.getLayer();

    //  Rsyn::PhysicalVia (LEF)
    if (rsynVias.empty()) {
        log() << "Error in " << __func__ << ": For " << name << " rsynVias is empty... " << std::endl;
    }

    int defaultViaTypeIdx = -1;
    const DBU dbuMax =
        std::numeric_limits<DBU>::has_infinity ? std::numeric_limits<DBU>::infinity() : std::numeric_limits<DBU>::max();
    std::tuple<DBU, DBU, DBU, DBU> bestScore(dbuMax, dbuMax, dbuMax, dbuMax);
    for (const Rsyn::PhysicalVia& rsynVia : rsynVias) {
        if (rsynVia.isViaDesign()) {
            continue;
        }

        if ((rsynVia.allBottomGeometries().size() != 1 ||
             rsynVia.allCutGeometries().size() != 1 ||
             rsynVia.allTopGeometries().size() != 1)) {
            if (setting.dbVerbose >= +VerboseLevelT::MIDDLE) {
                log() << "Warning in " << __func__ << ": For " << rsynVia.getName()
                      << " , has not exactly one metal layer bound or more than one cut layer bound... " << std::endl;
            }
            continue;
        }

        allViaTypes.emplace_back(rsynVia);
        const std::tuple<DBU, DBU, DBU, DBU>& score = allViaTypes.back().getDefaultScore(botDim, topDim);
        if (score < bestScore) {
            bestScore = score;
            defaultViaTypeIdx = allViaTypes.size() - 1;
        }
    }

    if (defaultViaTypeIdx == -1) {
        log() << "Error in " << __func__ << ": For " << name << " all rsyn vias have not exactly one via bound... "
              << std::endl;
    }

    // make default via the first one
    if (defaultViaTypeIdx > 0) {
        std::swap(allViaTypes[0], allViaTypes[defaultViaTypeIdx]);
    }
    // init ViaType::idx
    for (unsigned i = 0; i != allViaTypes.size(); ++i) {
        allViaTypes[i].idx = i;
    }
}

ostream& CutLayer::printBasics(ostream& os) const {
    os << name << ": idx=" << idx << ", viaTypes=" << defaultViaType().name << " (";
    for (auto via : allViaTypes) {
        if (via.name != defaultViaType().name) {
            os << via.name << " ";
        }
    }
    os << ")";
    return os;
}

ostream& CutLayer::printDesignRules(ostream& os) const {
    os << name << ": width=" << width << ", space=" << spacing;
    return os;
}

ostream& CutLayer::printViaOccupancyLUT(ostream& os) const {
    os << name << ": viaCut(" << viaCut().size() / 2 + 1 << ',' << viaCut()[0].size() / 2 + 1 << ")";
    os << ", viaMetal(" << viaMetal().size() / 2 + 1 << ',' << viaMetal()[0].size() / 2 + 1 << ")";
    // TODO: make xSize member variables, since they will be the same in a LUT over all cps
    auto getMaxSize = [](const vector<vector<vector<bool>>>& LUT, size_t& xSize, size_t& ySize) {
        xSize = 0;
        ySize = 0;
        for (const vector<vector<bool>>& cpLUT : LUT) {
            if (cpLUT.size()) {
                xSize = max(xSize, cpLUT.size());
                ySize = max(ySize, cpLUT[0].size());
            }
        }
    };
    size_t xSize, ySize;
    os << ", viaBotVia(";
    if (defaultViaType().allViaBotVia.size()) {
        getMaxSize(viaBotVia(), xSize, ySize);
        os << viaBotVia().size() << ',' << xSize << ',' << ySize << ")";
    }
    else os << "-,-,-)";
    os << ", viaTopVia(";
    if (defaultViaType().allViaTopVia.size()) {
        getMaxSize(viaTopVia(), xSize, ySize);
        os << viaTopVia().size() << ',' << xSize << ',' << ySize << ")";
    }
    else os << "-,-,-)";
    getMaxSize(viaBotWire(), xSize, ySize);
    os << ", viaBotWire(" << viaBotWire().size() << ',' << xSize << ',' << ySize << ")";
    getMaxSize(viaTopWire(), xSize, ySize);
    os << ", viaTopWire(" << viaTopWire().size() << ',' << xSize << ',' << ySize << ")";
    return os;
}

ostream& operator<<(ostream& os, const CutLayer& layer) { return layer.printBasics(os); }

}  //   namespace db
