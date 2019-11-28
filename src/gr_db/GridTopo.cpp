#include "GridTopo.h"

namespace gr {

void GrSteiner::setParent(std::shared_ptr<GrSteiner> childNode, std::shared_ptr<GrSteiner> parentNode) {
    parentNode->children.push_back(childNode);
    childNode->parent = parentNode;
}

void GrSteiner::resetParent(std::shared_ptr<GrSteiner> node) {
    assert(node->parent);

    auto& n = node->parent->children;
    auto it = find(n.begin(), n.end(), node);
    assert(it != n.end());
    *it = n.back();
    n.pop_back();

    node->parent.reset();
}

void GrSteiner::preOrder(std::shared_ptr<GrSteiner> node,
                         const std::function<void(std::shared_ptr<GrSteiner>)>& visit) {
    visit(node);
    for (auto c : node->children) preOrder(c, visit);
}

void GrSteiner::postOrder(std::shared_ptr<GrSteiner> node,
                          const std::function<void(std::shared_ptr<GrSteiner>)>& visit) {
    for (auto c : node->children) postOrder(c, visit);
    visit(node);
}

void GrSteiner::postOrderCopy(std::shared_ptr<GrSteiner> node,
                              const std::function<void(std::shared_ptr<GrSteiner>)>& visit) {
    auto tmp = node->children;
    for (auto c : tmp) postOrderCopy(c, visit);
    visit(node);
}

void GrSteiner::mergeNodes(std::shared_ptr<GrSteiner> root) {
    postOrderCopy(root, [](std::shared_ptr<GrSteiner> node) {
        // parent - node - child
        if (node->parent && node->parent->layerIdx == node->layerIdx && node->children.size() == 1 &&
            node->pinIdx < 0) {
            auto oldChild = node->children[0];
            if (node->layerIdx == oldChild->layerIdx && node->getPrefIdx() == oldChild->getPrefIdx() &&
                node->parent->getPrefIdx() == node->getPrefIdx()) {
                auto oldParent = node->parent;
                resetParent(node);
                resetParent(oldChild);
                setParent(oldChild, oldParent);
            }
        }
    });
}

void GrSteiner::removeRedundancy(std::shared_ptr<GrSteiner> root) {
    for (int i = 0; i < root->children.size(); i++) {
        if (GrPoint(*root) == GrPoint(*root->children[i])) {
            auto child_ptr = root->children[i];
            root->children.erase(root->children.begin() + i);
            i--;
            for (auto& grand_child : child_ptr->children) {
                setParent(grand_child, root);
            }
        }
    }
    for (auto& child : root->children) {
        removeRedundancy(child);
    }
}

bool GrSteiner::checkConnectivity(std::shared_ptr<GrSteiner> root, vector<GrPoint>& pins) {
    bool detached_node = false;
    std::unordered_map<GrPoint, bool> pin_visited;
    for (int i = 0; i < pins.size(); i++) {
        pin_visited[pins[i]] = false;
    }
    preOrder(root, [&](std::shared_ptr<GrSteiner> node) {
        if (pin_visited.find(GrPoint(*node)) != pin_visited.end()) {
            node->pinIdx = 1;
            pin_visited[GrPoint(*node)] = true;
        }
        for (auto& child : node->children) {
            // If two nodes are connected, at least 2 of their coordinates should be the same
            if (node->layerIdx == child->layerIdx) {
                if (node->x != child->x && node->y != child->y) {
                    detached_node = true;
                }
            } else if (abs(node->layerIdx - child->layerIdx) <= 1) {
                if (node->x != child->x || node->y != child->y) {
                    detached_node = true;
                }
            } else {
                detached_node = true;
            }
            int match_cnt = 0;
            if (node->x == child->x) match_cnt++;
            if (node->y == child->y) match_cnt++;
            if (node->layerIdx == child->layerIdx) match_cnt++;
            if (match_cnt < 2) detached_node = true;
        }
    });
    for (auto& p_v : pin_visited) {
        if (p_v.second == false) return false;
    }
    if (detached_node) return false;
    return true;
}

ostream& operator<<(ostream& os, const GrSteiner& node) {
    os << GrPoint(node);
    if (node.pinIdx >= 0) os << " pin" << node.pinIdx;
    return os;
}

void GrSteiner::printTree(ostream& os, int depth) {
    os << *this << ", ";
    bool first = true;
    for (auto child : children) {
        if (!first) {
            for (int i = 0; i < depth; ++i) os << '.';
            os << *this << ", ";
        }
        child->printTree(os, depth + 1);
        if (children.size() > 1) {
            os << std::endl;
        }
        first = false;
    }
}

}  // namespace gr
