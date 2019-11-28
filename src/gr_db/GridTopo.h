#pragma once

#include "GrGeoPrimitive.h"

namespace gr {

class GrSteiner : public GrPoint {
public:
    int pinIdx;  // -1 stands for "not pin"
    std::shared_ptr<GrSteiner> parent;
    vector<std::shared_ptr<GrSteiner>> children;

    GrSteiner(const GrPoint& grPoint, int pinIndex = -1) : GrPoint(grPoint), pinIdx(pinIndex) {}
    bool isRealPin() const { return pinIdx >= 0; }

    // Set/reset parent
    static void setParent(std::shared_ptr<GrSteiner> childNode, std::shared_ptr<GrSteiner> parentNode);
    static void resetParent(std::shared_ptr<GrSteiner> node);

    // Traverse
    static void preOrder(std::shared_ptr<GrSteiner> node, const std::function<void(std::shared_ptr<GrSteiner>)>& visit);
    static void postOrder(std::shared_ptr<GrSteiner> node,
                          const std::function<void(std::shared_ptr<GrSteiner>)>& visit);
    static void postOrderCopy(std::shared_ptr<GrSteiner> node,
                              const std::function<void(std::shared_ptr<GrSteiner>)>& visit);

    // Merge two same-layer edges (assume they are on the same track)
    static void mergeNodes(std::shared_ptr<GrSteiner> root);

    // Remove redundant nodes
    static void removeRedundancy(std::shared_ptr<GrSteiner> root);
    static bool checkConnectivity(std::shared_ptr<GrSteiner> root, vector<GrPoint>& pins);

    friend ostream& operator<<(ostream& os, const GrSteiner& node);
    void printTree(ostream& os = std::cout, int depth = 0);
};

}  // namespace gr
