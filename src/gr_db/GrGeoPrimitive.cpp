#include "GrGeoPrimitive.h"

namespace gr {
// GrPoint

bool GrPoint::operator==(const GrPoint& rhs) const { return layerIdx == rhs.layerIdx && x == rhs.x && y == rhs.y; }

bool GrPoint::operator!=(const GrPoint& rhs) const { return layerIdx != rhs.layerIdx || x != rhs.x || y != rhs.y; }

ostream& operator<<(ostream& os, const GrPoint& gp) {
    os << "gPt(l=" << gp.layerIdx << ", xIdx=" << gp.x << ", yIdx=" << gp.y << ")";
    return os;
}

// PointOnLayer

bool PointOnLayer::operator==(const PointOnLayer& rhs) const {
    return layerIdx == rhs.layerIdx && x == rhs.x && y == rhs.y;
}

bool PointOnLayer::operator!=(const PointOnLayer& rhs) const {
    return layerIdx != rhs.layerIdx || x != rhs.x || y != rhs.y;
}

ostream& operator<<(ostream& os, const PointOnLayer& gp) {
    os << "gPt(l=" << gp.layerIdx << ", xIdx=" << gp.x << ", yIdx=" << gp.y << ")";
    return os;
}

// GrEdge

ostream& operator<<(ostream& os, const GrEdge& edge) {
    os << "gEdge(" << edge.u << " " << edge.v << ")";
    return os;
}

bool GrEdge::operator==(const GrEdge& rhs) const { return u == rhs.u && v == rhs.v; }

// GrBoxOnLayer

bool GrBoxOnLayer::operator==(const GrBoxOnLayer& rhs) const {
    return layerIdx == rhs.layerIdx && x == rhs.x && y == rhs.y;
}

ostream& operator<<(ostream& os, const GrBoxOnLayer& gb) {
    os << "gBox(l=" << gb.layerIdx << ", xIdx=" << gb.x << ", yIdx=" << gb.y << ")";
    return os;
}

// slice polygons along sliceDir
// sliceDir: 0 for x/vertical, 1 for y/horizontal
void GrBoxOnLayer::sliceGrPolygons(vector<GrBoxOnLayer>& boxes, bool mergeAdj) {
    if (boxes.size() <= 1) return;

    auto dir = database.getLayerDir(boxes[0].layerIdx);

    vector<int> locs;
    for (const auto& box : boxes) {
        locs.push_back(box[dir].low);
        locs.push_back(box[dir].high);
    }
    sort(locs.begin(), locs.end());
    locs.erase(unique(locs.begin(), locs.end()), locs.end());

    // slice each box
    vector<GrBoxOnLayer> slicedBoxes;
    for (const auto& box : boxes) {
        GrBoxOnLayer slicedBox = box;
        auto itLoc = lower_bound(locs.begin(), locs.end(), box[dir].low);
        auto itEnd = upper_bound(itLoc, locs.end(), box[dir].high);
        slicedBox[dir].Set(*itLoc);
        slicedBoxes.push_back(slicedBox);  // front boundary
        while ((itLoc + 1) != itEnd) {
            int left = *itLoc, right = *(itLoc + 1);
            if ((right - left) > 1) {
                slicedBox[dir].Set(left + 1, right - 1);
                slicedBoxes.push_back(slicedBox);  // middle
            }
            slicedBox[dir].Set(right);
            slicedBoxes.push_back(slicedBox);  // back boundary
            ++itLoc;
        }
    }
    boxes = move(slicedBoxes);

    // merge overlaped boxes over crossPoints
    utils::MergeRects(boxes, 1 - dir);

    // stitch boxes over tracks
    utils::MergeRects(boxes, dir);

    if (mergeAdj) {
        // merge box on adjacent gridline
        std::sort(boxes.begin(), boxes.end(), [&](const BoxT& lhs, const BoxT& rhs) {
            return lhs[dir].low < rhs[dir].low || (lhs[dir].low == rhs[dir].low && lhs[1 - dir].low < rhs[1 - dir].low);
        });
        std::vector<GrBoxOnLayer> mergedBoxes;
        mergedBoxes.push_back(boxes.front());
        for (int i = 1; i < boxes.size(); ++i) {
            auto& lastBox = mergedBoxes.back();
            auto& slicedBox = boxes[i];
            if (abs(slicedBox[dir].high - lastBox[dir].low) <= 1 && slicedBox[1 - dir] == lastBox[1 - dir]) {
                lastBox[dir] = lastBox[dir].UnionWith(slicedBox[dir]);
            } else {  // neither misaligned not seperated
                mergedBoxes.push_back(slicedBox);
            }
        }

        boxes = move(mergedBoxes);
    }
}

}  // namespace gr
