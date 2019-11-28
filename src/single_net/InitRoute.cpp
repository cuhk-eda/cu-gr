#include "InitRoute.h"

using namespace std;

mutex fluteMutex;

struct hash_tuple {  // hash binary tuple
    template <class T>
    size_t operator()(const tuple<T, T>& tup) const {
        auto hash1 = hash<T>{}(get<0>(tup));
        auto hash2 = hash<T>{}(get<1>(tup));
        return hash1 ^ hash2;
    }
};

void InitRoute::runFlute() {
    // get net center
    // float net_ctrz = 0;
    float net_ctrx = 0;
    float net_ctry = 0;
    for (auto& pinBoxes : grNet.pinAccessBoxes) {
        // float pin_ctrz = 0;
        float pin_ctrx = 0;
        float pin_ctry = 0;
        for (auto& pinBox : pinBoxes) {
            pin_ctrx += pinBox.x;
            pin_ctry += pinBox.y;
        }
        pin_ctrx /= pinBoxes.size();
        pin_ctry /= pinBoxes.size();

        net_ctrx += pin_ctrx;
        net_ctry += pin_ctry;
    }
    net_ctrx /= grNet.pinAccessBoxes.size();
    net_ctry /= grNet.pinAccessBoxes.size();

    // get pin center
    vector<tuple<int, int>> pinCenters;

    for (auto& pinBoxes : grNet.pinAccessBoxes) {
        float xCenter = 0;
        float yCenter = 0;
        for (auto& pinBox : pinBoxes) {
            xCenter += pinBox.x;
            yCenter += pinBox.y;
        }
        xCenter /= pinBoxes.size();
        yCenter /= pinBoxes.size();
        // 3 kinds of accessibility (0) totally vio-free (1) one side vio-free (2) no side vio-free
        float min_dist[3];
        min_dist[0] = min_dist[1] = min_dist[2] = numeric_limits<float>::max();
        int best_box[3] = {-1, -1, -1};

        for (int pb = 0; pb < pinBoxes.size(); pb++) {
            // check pin box's accessibility
            auto& pb_x = pinBoxes[pb].x;
            auto& pb_y = pinBoxes[pb].y;
            int layer_idx = pinBoxes[pb].layerIdx;
            int is_x = database.getLayerDir(layer_idx) == X ? 1 : 0;
            auto low_edge =
                gr::GrEdge({layer_idx, max(pb_x - (1 - is_x), 0), max(pb_y - is_x, 0)}, {layer_idx, pb_x, pb_y});
            auto high_edge = gr::GrEdge({layer_idx, pb_x, pb_y},
                                        {layer_idx,
                                         min(pb_x + (1 - is_x), grDatabase.getNumGrPoint(X) - 1),
                                         min(pb_y + is_x, grDatabase.getNumGrPoint(Y) - 1)});

            int pb_access = 0;
            pb_access += int(grDatabase.hasVio(low_edge, false));
            pb_access += int(grDatabase.hasVio(high_edge, false));

            float dist = abs(pinBoxes[pb].x - net_ctrx) + abs(pinBoxes[pb].y - net_ctrx);
            if (dist < min_dist[pb_access]) {
                min_dist[pb_access] = dist;
                best_box[pb_access] = pb;
            }
        }
        for (int ac = 0; ac < 3; ac++) {
            if (best_box[ac] != -1) {
                pinCenters.emplace_back(make_tuple(pinBoxes[best_box[ac]].x, pinBoxes[best_box[ac]].y));
                break;
            }
        }
    }

    // location to pins
    unordered_map<tuple<int, int>, vector<int>, hash_tuple> loc2Pins;
    for (int i = 0; i < pinCenters.size(); i++) {
        loc2Pins[pinCenters[i]].emplace_back(i);
    }

    // flute
    int degree = loc2Pins.size();
    int xs[100 * degree];
    int ys[100 * degree];
    int pt_cnt = 0;
    int node_cnt = 0;
    // if (loc2Pins.size() > MAX_DEGREE) {
    //     log() << "Warning: Net degree larger than MAXD(flute)" << endl;
    // }
    for (auto& loc2pin : loc2Pins) {
        const tuple<int, int>& loc = loc2pin.first;
        xs[pt_cnt] = get<0>(loc);
        ys[pt_cnt] = get<1>(loc);
        pt_cnt++;
    }
    if (degree >= 2) {
        fluteMutex.lock();
        Tree flutetree = flute(degree, xs, ys, ACCURACY);
        fluteMutex.unlock();
        unordered_map<tuple<int, int>, int, hash_tuple> loc2Node;  // location -> RouteNode index
        for (int i = 0; i < degree * 2 - 2; i++) {
            Branch& branch1 = flutetree.branch[i];
            Branch& branch2 = flutetree.branch[branch1.n];
            tuple<int, int> fluteEdge[2];
            fluteEdge[0] = make_tuple(branch1.x, branch1.y);
            fluteEdge[1] = make_tuple(branch2.x, branch2.y);
            // create route nodes
            for (int j = 0; j < 2; j++) {
                tuple<int, int>& nodeLoc = fluteEdge[j];
                if (loc2Node.find(nodeLoc) == loc2Node.end()) {
                    RouteNode node(fluteEdge[j]);
                    // pin info
                    if (loc2Pins.find(nodeLoc) != loc2Pins.end()) {
                        node.pinIdxs = loc2Pins[nodeLoc];
                        for (int pIdx : node.pinIdxs) {
                            int layerIdx = grNet.pinAccessBoxes[pIdx][0].layerIdx;
                            node.pinLayers.emplace_back(layerIdx);

                            // for (auto& box: grNet.pinAccessBoxes[pIdx]) {
                            //     if (box.layerIdx != layerIdx) {
                            //         log() << "Error: Pin across layers spotted" << endl;
                            //     }
                            // }
                        }
                    }
                    node.idx = node_cnt;
                    routeNodes[node_cnt++] = node;
                    loc2Node[nodeLoc] = routeNodes.size() - 1;
                }
            }
            // add connectivity info
            if (fluteEdge[0] != fluteEdge[1]) {
                int nodeIdx1 = loc2Node[fluteEdge[0]];
                int nodeIdx2 = loc2Node[fluteEdge[1]];
                routeNodes[nodeIdx1].toConnect.insert(nodeIdx2);
                routeNodes[nodeIdx2].toConnect.insert(nodeIdx1);
            }
        }
    } else if (degree == 1) {
        auto nodeLoc = make_tuple(xs[0], ys[0]);
        RouteNode node(nodeLoc);
        if (loc2Pins.find(nodeLoc) != loc2Pins.end()) {
            node.pinIdxs = loc2Pins[nodeLoc];
            for (int pIdx : node.pinIdxs) {
                int layerIdx = grNet.pinAccessBoxes[pIdx][0].layerIdx;
                node.pinLayers.emplace_back(layerIdx);
            }
        } else {
            log() << "Error: loc2Pins is empty." << std::endl;
        }
        node.idx = node_cnt;
        routeNodes[node_cnt++] = node;
        return;
    } else {
        log() << "Error: degree = 0." << std::endl;
    }
}

void InitRoute::patternRoute() {
    int layer_cnt = database.getLayerNum();
    for (auto& edge : routeEdges) {
        auto& fromNode = routeNodes[edge.from];
        auto& toNode = routeNodes[edge.to];
        // Initialize fromNode's exitCosts
        fromNode.exitCosts.resize(layer_cnt, numeric_limits<db::CostT>::max());
        int lowest_pin = layer_cnt;
        int highest_pin = -1;
        for (int pin_layer : fromNode.pinLayers) {
            lowest_pin = min(lowest_pin, pin_layer);
            highest_pin = max(highest_pin, pin_layer);
        }
        if (fromNode.childIdxs.size() == 0) {  // no children nodes, only via costs
            for (int layer_idx = 0; layer_idx < layer_cnt; layer_idx++) {
                int lowest_layer = min(lowest_pin, layer_idx);
                int highest_layer = max(highest_pin, layer_idx);
                db::CostT exit_cost = grDatabase.getStackViaCost(gr::GrPoint(lowest_layer, fromNode.x, fromNode.y),
                                                                 highest_layer - lowest_layer);
                fromNode.exitCosts[layer_idx] = exit_cost;
            }
        } else {
            // initialize exitEnterLayers map
            for (int child_idx : fromNode.childIdxs) {
                fromNode.exitEnterLayers[child_idx] = vector<int>(layer_cnt, -1);
            }
            // enumerate all children layer combinations
            int num_children = fromNode.childIdxs.size();
            for (int enum_idx = 0; enum_idx < pow(layer_cnt, num_children); enum_idx++) {
                vector<int> enum_vec;
                int e_idx = enum_idx;
                for (int en = 0; en < num_children; en++) {
                    enum_vec.emplace_back(e_idx % layer_cnt);
                    e_idx /= layer_cnt;
                }
                db::CostT prev_cost = 0;
                for (int c = 0; c < num_children; c++) {
                    int child_idx = fromNode.childIdxs[c];
                    prev_cost += fromNode.enterCosts[child_idx][enum_vec[c]];
                }
                int highest_layer = highest_pin;
                int lowest_layer = lowest_pin;
                for (int e_layer : enum_vec) {
                    highest_layer = max(highest_layer, e_layer);
                    lowest_layer = min(lowest_layer, e_layer);
                }
                for (int layer_idx = 0; layer_idx < layer_cnt; layer_idx++) {
                    int via_bottom = min(lowest_layer, layer_idx);
                    int via_height = max(highest_layer, layer_idx) - via_bottom;
                    db::CostT via_cost =
                        grDatabase.getStackViaCost(gr::GrPoint(via_bottom, fromNode.x, fromNode.y), via_height);
                    db::CostT cost = prev_cost + via_cost;
                    if (cost < fromNode.exitCosts[layer_idx]) {
                        fromNode.exitCosts[layer_idx] = cost;
                        for (int c = 0; c < num_children; c++) {
                            fromNode.exitEnterLayers[fromNode.childIdxs[c]][layer_idx] = enum_vec[c];
                        }
                    }
                }
            }
        }
        // Patterns
        LShape(edge);
    }
}

db::CostT InitRoute::getBufferedWireCost(gr::GrEdge edge) {
    if (wireCostBuffer.find(edge) != wireCostBuffer.end()) {
        return wireCostBuffer[edge];
    } else {
        auto wireCost = grDatabase.getWireCost(edge);
        wireCostBuffer[edge] = wireCost;
        return wireCost;
    }
}

void InitRoute::LShape(const RouteEdge& edge) {
    int layer_cnt = database.getLayerNum();

    auto& fromNode = routeNodes[edge.from];
    auto& toNode = routeNodes[edge.to];

    vector<db::CostT> enter_cost(layer_cnt, numeric_limits<db::CostT>::max());
    vector<vector<gr::GrPoint>> enter_edges(layer_cnt);

    if (fromNode.x == toNode.x || fromNode.y == toNode.y) {  // has no bending point
        bool dir = (fromNode.x == toNode.x ? X : Y);
        for (int layer = 0; layer < layer_cnt; layer++) {
            if (database.getLayerDir(layer) != dir) continue;
            gr::GrEdge gr_edge(gr::GrPoint(layer, fromNode.x, fromNode.y), gr::GrPoint(layer, toNode.x, toNode.y));
            db::CostT edge_cost = grDatabase.getWireCost(gr_edge);
            db::CostT cost = fromNode.exitCosts[layer] + edge_cost;
            if (cost < enter_cost[layer]) {
                enter_cost[layer] = cost;
                vector<gr::GrPoint> edge_vec;
                edge_vec.emplace_back(gr::GrPoint(layer, fromNode.x, fromNode.y));
                // edge_vec.emplace_back(gr::GrPoint(layer+1, fromNode.x, fromNode.y)); // additional
                // edge_vec.emplace_back(gr::GrPoint(layer, fromNode.x, fromNode.y)); // additional
                edge_vec.emplace_back(gr::GrPoint(layer, toNode.x, toNode.y));
                // edge_vec.emplace_back(gr::GrPoint(layer, toNode.x, toNode.y)); // additional
                // edge_vec.emplace_back(gr::GrPoint(layer, toNode.x, toNode.y)); // additional
                enter_edges[layer] = move(edge_vec);
            }
        }
    } else {  // has a bending point
        for (int to_layer = 0; to_layer < layer_cnt; to_layer++) {
            Dimension to_dir = database.getLayerDir(to_layer);
            int bend_x = -1;  // bending point
            int bend_y = -1;
            if (to_dir == Y) {
                bend_x = fromNode.x;
                bend_y = toNode.y;
            } else {
                bend_x = toNode.x;
                bend_y = fromNode.y;
            }

            // to_layer edge cost
            gr::GrEdge to_gr_edge(gr::GrPoint(to_layer, bend_x, bend_y), gr::GrPoint(to_layer, toNode.x, toNode.y));
            db::CostT to_edge_cost = grDatabase.getWireCost(to_gr_edge);

            db::CostT min_cost = numeric_limits<db::CostT>::max();
            int best_from_layer = -1;
            for (int from_layer = 0; from_layer < layer_cnt; from_layer++) {
                if (database.getLayerDir(from_layer) == to_dir) continue;  // same routing direction

                // from_layer edge cost
                gr::GrEdge from_gr_edge(gr::GrPoint(from_layer, fromNode.x, fromNode.y),
                                        gr::GrPoint(from_layer, bend_x, bend_y));
                db::CostT from_edge_cost = getBufferedWireCost(from_gr_edge);

                db::CostT bend_via_cost = grDatabase.getStackViaCost(
                    gr::GrPoint(min(from_layer, to_layer), bend_x, bend_y), abs(to_layer - from_layer));

                db::CostT cost = fromNode.exitCosts[from_layer] + from_edge_cost + bend_via_cost + to_edge_cost;
                if (cost < min_cost) {
                    min_cost = cost;
                    best_from_layer = from_layer;
                }
            }
            enter_cost[to_layer] = min_cost;
            vector<gr::GrPoint> edge_vec;
            edge_vec.emplace_back(gr::GrPoint(best_from_layer, fromNode.x, fromNode.y));
            bool go_up = (best_from_layer < to_layer);
            for (int cur_layer = best_from_layer;;) {
                edge_vec.emplace_back(gr::GrPoint(cur_layer, bend_x, bend_y));
                if (cur_layer == to_layer) break;
                cur_layer += (go_up ? 1 : -1);
            }
            // edge_vec.emplace_back(gr::GrPoint(best_from_layer, bend_x, bend_y));
            // edge_vec.emplace_back(gr::GrPoint(to_layer, bend_x, bend_y));
            edge_vec.emplace_back(gr::GrPoint(to_layer, toNode.x, toNode.y));
            enter_edges[to_layer] = move(edge_vec);
        }
    }
    toNode.enterCosts[edge.from] = move(enter_cost);
    toNode.enterEdges[edge.from] = move(enter_edges);
}


void InitRoute::buildTopo() {
    int layer_cnt = database.getLayerNum();
    int rootIdx = -1;
    int root_enter_layer = -1;
    // build topo tree
    if (routeEdges.size() > 0) {
        rootIdx = routeEdges[routeEdges.size() - 1].to;
        RouteNode& rootNode = routeNodes[rootIdx];

        int highest_layer = -1;
        int lowest_layer = layer_cnt;
        for (int pin_layer : rootNode.pinLayers) {
            lowest_layer = min(lowest_layer, pin_layer);
            highest_layer = max(highest_layer, pin_layer);
        }
        // find min cost enter layer
        assert(rootNode.enterCosts.size() == 1);
        auto& enterCosts = rootNode.enterCosts.begin()->second;
        db::CostT min_cost = numeric_limits<db::CostT>::max();
        for (int layer_idx = 0; layer_idx < layer_cnt; layer_idx++) {
            db::CostT cost = enterCosts[layer_idx];
            int via_bottom = min(lowest_layer, layer_idx);
            int via_height = max(highest_layer, layer_idx) - via_bottom;
            db::CostT via_cost =
                grDatabase.getStackViaCost(gr::GrPoint(via_bottom, rootNode.x, rootNode.y), via_height);
            cost += via_cost;  // via cost
            if (cost < min_cost) {
                min_cost = cost;
                root_enter_layer = layer_idx;
            }
        }
    } else {  // local net
        rootIdx = 0;
        root_enter_layer = routeNodes[rootIdx].pinLayers[0];
    }

    // log() << "cost " << min_cost << endl;

    function<shared_ptr<gr::GrSteiner>(int, int)> buildTopo = [&](int root_idx, int exit_layer) {
        RouteNode& rNode = routeNodes[root_idx];
        shared_ptr<gr::GrSteiner> topo_root = make_shared<gr::GrSteiner>(gr::GrPoint(exit_layer, rNode.x, rNode.y), -1);

        // build layer topo
        int low_layer = exit_layer;
        int high_layer = exit_layer;

        for (int child_idx : rNode.childIdxs) {
            int child_enter_layer = exit_layer;  // root has no exit, thus no exitEnterLayers map, we use exit layer to
                                                 // indicate its child's enter layer
            if (rNode.exitEnterLayers.size() > 0) {
                child_enter_layer = rNode.exitEnterLayers[child_idx][exit_layer];
            }
            low_layer = min(low_layer, child_enter_layer);
            high_layer = max(high_layer, child_enter_layer);
        }

        for (int pin_layer : rNode.pinLayers) {
            if (pin_layer != exit_layer) {
                low_layer = min(low_layer, pin_layer);
                high_layer = max(high_layer, pin_layer);
            }
        }

        unordered_map<int, shared_ptr<gr::GrSteiner>> layer2layerTopo;
        layer2layerTopo[exit_layer] = topo_root;
        for (int cur_layer = exit_layer + 1; cur_layer <= high_layer; cur_layer++) {
            layer2layerTopo[cur_layer] = make_shared<gr::GrSteiner>(gr::GrPoint(cur_layer, rNode.x, rNode.y), -1);
            gr::GrSteiner::setParent(layer2layerTopo[cur_layer], layer2layerTopo[cur_layer - 1]);
        }
        for (int cur_layer = exit_layer - 1; cur_layer >= low_layer; cur_layer--) {
            layer2layerTopo[cur_layer] = make_shared<gr::GrSteiner>(gr::GrPoint(cur_layer, rNode.x, rNode.y), -1);
            gr::GrSteiner::setParent(layer2layerTopo[cur_layer], layer2layerTopo[cur_layer + 1]);
        }

        for (int child_idx : rNode.childIdxs) {  // build branches
            int child_enter_layer = exit_layer;
            if (rNode.exitEnterLayers.size() > 0) {
                child_enter_layer = rNode.exitEnterLayers[child_idx][exit_layer];
            }
            auto& enter_edge = rNode.enterEdges[child_idx][child_enter_layer];
            if (enter_edge.size() == 0) {
                log() << "Error: Enter edge missing" << endl;
            }

            int child_exit_layer = enter_edge[0].layerIdx;
            shared_ptr<gr::GrSteiner> child_topo = buildTopo(child_idx, child_exit_layer);
            for (auto& point : enter_edge) {
                shared_ptr<gr::GrSteiner> steiner = make_shared<gr::GrSteiner>(point, -1);
                gr::GrSteiner::setParent(child_topo, steiner);
                child_topo = steiner;
            }
            // if (gr::GrPoint(*child_topo) !=  gr::GrPoint(*topo_root)) {
            gr::GrSteiner::setParent(child_topo, layer2layerTopo[child_enter_layer]);
            // } else {
            //     gr::GrSteiner::setParent(child_topo->children[0], topo_root);
            // }
        }

        return topo_root;
    };

    // dumpTo grNet
    grNet.gridTopo.emplace_back(buildTopo(rootIdx, root_enter_layer));
    gr::GrSteiner::removeRedundancy(grNet.gridTopo[0]);

    vector<gr::GrPoint> pin_locs;
    for (auto& kv : routeNodes) {
        auto& node = kv.second;
        for (int pin_layer : node.pinLayers) {
            // cout << "(" << pin_layer << "," << node.x << "," << node.y << ") ";
            pin_locs.emplace_back(gr::GrPoint(pin_layer, node.x, node.y));
        }
    }

    // debug

    // cout << endl;
    // log() << endl;
    // grNet.preOrderVisitGridTopo([&](std::shared_ptr<gr::GrSteiner> topo) {
    //     if (topo->children.size() == 0) return;
    //     log() << gr::GrPoint(*topo) << "->";
    //     for(auto& child: topo->children) {
    //         cout << gr::GrPoint(*child) << " ";
    //     }
    //     cout << endl;
    // });
    // log() << "cost " << min_cost << endl;
    // log() << endl << "+ + + + + + + + + + + + + + + + + + + + + +" << endl;

    // Check Connectivity and Mark Pins (0: no pin; 1: has pin)
    if (!gr::GrSteiner::checkConnectivity(grNet.gridTopo[0], pin_locs)) {
        log() << "Error: Connectivity check failed" << endl;
    }
}

void InitRoute::plan_fluteOnly() {
    runFlute();

    int s = routeNodes.begin()->first;
    queue<int> tmp_q;
    tmp_q.push(s);
    set<int> vis;
    while (tmp_q.empty() == false) {
        int u = tmp_q.front();
        tmp_q.pop();
        vis.insert(u);
        RouteNode& node = routeNodes[u];
        for (int nIdx : node.toConnect) {
            if (vis.find(nIdx) != vis.end()) continue;
            addUsage2D(node, routeNodes[nIdx], 1);  // update the usage
            tmp_q.push(nIdx);
        }
    }
}
void InitRoute::edge_shift2d(std::map<int, RouteNode>& cur_routeNodes) {
    if (cur_routeNodes.size() == 1) return;
    vector<RouteEdge> edgeset;
    map<pair<int, int>, set<int>> loc_nodes;  // will be used to merge overlapping nodes
    int s = cur_routeNodes.begin()->first;
    queue<int> tmp_q;
    tmp_q.push(s);
    set<int> vis;

    while (tmp_q.empty() == false) {
        int u = tmp_q.front();
        tmp_q.pop();
        vis.insert(u);
        RouteNode& node = cur_routeNodes[u];
        loc_nodes[make_pair(node.x, node.y)].insert(u);

        for (int nIdx : node.toConnect) {
            if (vis.find(nIdx) != vis.end()) continue;
            RouteEdge edge;
            edge.from = nIdx;
            edge.to = u;
            edgeset.emplace_back(edge);
            removeUsage2D(node, cur_routeNodes[nIdx], 1);  // Remove this net
            tmp_q.push(nIdx);
        }
    }

    auto getStraightCost = [&](int dir, int gridline, utils::IntervalT<int> range) {
        double cost = 0;
        for (int cp = range.low; cp < range.high; cp++) {
            cost += grDatabase.getCost2D(dir, gridline, cp);
        }
        return cost;
    };

    std::function<double(int, int, int, int)> getCost =
        [&](int x1, int y1, int x2, int y2) {  // GR point (x1,y1) -> (x2,y2)
            double ret = 0;
            int layer_dir_count = 0;
            if (x1 == x2 || y1 == y2) {  // straight line
                int dir = (x1 == x2 ? X : Y);
                if (dir == X) {
                    utils::IntervalT<int> Range(min(y1, y2), max(y1, y2));
                    ret = getStraightCost(X, x1, Range);
                } else {
                    utils::IntervalT<int> Range(min(x1, x2), max(x1, x2));
                    ret = getStraightCost(Y, y1, Range);
                }
            } else {  // choose the L with lower cost
                int x3 = x1;
                int y3 = y2;
                int x4 = x2;
                int y4 = y1;
                ret = min(getCost(x1, y1, x3, y3) + getCost(x2, y2, x3, y3),
                          getCost(x1, y1, x4, y4) + getCost(x2, y2, x4, y4));
            }
            return ret;
        };

    auto shiftEdge = [&](bool& shifted) {
        int find_edge = -1;
        int dir = -1;
        bool extendSafeRange = false;

        for (int i = 0; i < edgeset.size(); i++) {
            auto& edge = edgeset[i];
            auto& fromNode = cur_routeNodes[edge.from];
            auto& toNode = cur_routeNodes[edge.to];
            if (fromNode.pinIdxs.size() == 0 && toNode.pinIdxs.size() == 0 && fromNode.degree() == 3 &&
                toNode.degree() == 3) {  // two steiner points encountered,require the degree to be 3
                if (fromNode.x == toNode.x || fromNode.y == toNode.y) {  // vertical or horizontal
                    dir = (fromNode.x == toNode.x ? X : Y);
                    vector<int> fromNeighbor;  // the two neighbors of fromNode
                    vector<int> toNeighbor;    // the two neighbors of toNode
                    for (auto& nei : fromNode.toConnect) {
                        if (nei != edge.to) {
                            fromNeighbor.push_back(nei);
                        }
                    }
                    for (auto& nei : toNode.toConnect) {
                        if (nei != edge.from) {
                            toNeighbor.push_back(nei);
                        }
                    }
                    assert(fromNeighbor.size() == 2);
                    assert(toNeighbor.size() == 2);
                    if (dir == X) {  // vertical edge

                        if (cur_routeNodes[fromNeighbor[0]].x > cur_routeNodes[fromNeighbor[1]].x) {
                            swap(fromNeighbor[0], fromNeighbor[1]);
                        }
                        if (cur_routeNodes[toNeighbor[0]].x > cur_routeNodes[toNeighbor[1]].x) {
                            swap(toNeighbor[0], toNeighbor[1]);
                        }

                        utils::IntervalT<int> r1(cur_routeNodes[fromNeighbor[0]].x, cur_routeNodes[fromNeighbor[1]].x);
                        utils::IntervalT<int> r2(cur_routeNodes[toNeighbor[0]].x, cur_routeNodes[toNeighbor[1]].x);
                        utils::IntervalT<int> safeRange = r1.IntersectWith(r2);
                        if (!safeRange.IsStrictValid()) {
                            continue;
                        }
                        utils::IntervalT<int> yRange(min(fromNode.y, toNode.y), max(fromNode.y, toNode.y));

                        int best_x = -1;
                        double best_cost = std::numeric_limits<double>::infinity();

                        for (int cur_x = safeRange.low; cur_x <= safeRange.high; cur_x++) {
                            // try each candidate and choose the one with smallest cost
                            double cur_cost = getStraightCost(X, cur_x, yRange);
                            cur_cost += getCost(cur_x,
                                                fromNode.y,
                                                cur_routeNodes[fromNeighbor[0]].x,
                                                cur_routeNodes[fromNeighbor[0]].y) +
                                        getCost(cur_x,
                                                fromNode.y,
                                                cur_routeNodes[fromNeighbor[1]].x,
                                                cur_routeNodes[fromNeighbor[1]].y);
                            cur_cost +=
                                getCost(
                                    cur_x, toNode.y, cur_routeNodes[toNeighbor[0]].x, cur_routeNodes[toNeighbor[0]].y) +
                                getCost(
                                    cur_x, toNode.y, cur_routeNodes[toNeighbor[1]].x, cur_routeNodes[toNeighbor[1]].y);
                            if (cur_cost < best_cost) {
                                best_cost = cur_cost;
                                best_x = cur_x;
                            }
                        }
                        if (best_x != -1 && fromNode.x != best_x) {  // need to move, update the topology and locations

                            loc_nodes[make_pair(fromNode.x, fromNode.y)].erase(fromNode.idx);
                            loc_nodes[make_pair(toNode.x, toNode.y)].erase(toNode.idx);
                            fromNode.x = best_x;
                            toNode.x = best_x;
                            loc_nodes[make_pair(fromNode.x, fromNode.y)].insert(fromNode.idx);
                            loc_nodes[make_pair(toNode.x, toNode.y)].insert(toNode.idx);
                            shifted = true;
                            break;
                        } else {  // check next candidate
                            continue;
                        }
                    } else {
                        if (cur_routeNodes[fromNeighbor[0]].y > cur_routeNodes[fromNeighbor[1]].y) {
                            swap(fromNeighbor[0], fromNeighbor[1]);
                        }
                        if (cur_routeNodes[toNeighbor[0]].y > cur_routeNodes[toNeighbor[1]].y) {
                            swap(toNeighbor[0], toNeighbor[1]);
                        }
                        utils::IntervalT<int> r1(cur_routeNodes[fromNeighbor[0]].y, cur_routeNodes[fromNeighbor[1]].y);
                        utils::IntervalT<int> r2(cur_routeNodes[toNeighbor[0]].y, cur_routeNodes[toNeighbor[1]].y);
                        utils::IntervalT<int> safeRange = r1.IntersectWith(r2);
                        if (!safeRange.IsStrictValid()) {
                            continue;
                        }
                        utils::IntervalT<int> xRange(min(fromNode.x, toNode.x), max(fromNode.x, toNode.x));
                        int best_y = -1;
                        double best_cost = std::numeric_limits<double>::infinity();

                        for (int cur_y = safeRange.low; cur_y <= safeRange.high; cur_y++) {
                            // try each candidate and choose the one with smallest cost
                            double cur_cost = getStraightCost(Y, cur_y, xRange);
                            cur_cost += getCost(fromNode.x,
                                                cur_y,
                                                cur_routeNodes[fromNeighbor[0]].x,
                                                cur_routeNodes[fromNeighbor[0]].y) +
                                        getCost(fromNode.x,
                                                cur_y,
                                                cur_routeNodes[fromNeighbor[1]].x,
                                                cur_routeNodes[fromNeighbor[1]].y);
                            cur_cost +=
                                getCost(
                                    toNode.x, cur_y, cur_routeNodes[toNeighbor[0]].x, cur_routeNodes[toNeighbor[0]].y) +
                                getCost(
                                    toNode.x, cur_y, cur_routeNodes[toNeighbor[1]].x, cur_routeNodes[toNeighbor[1]].y);
                            if (cur_cost < best_cost) {
                                best_cost = cur_cost;
                                best_y = cur_y;
                            }
                        }
                        if (best_y != -1 && fromNode.y != best_y) {  // need to move, update the topology and locations
                            loc_nodes[make_pair(fromNode.x, fromNode.y)].erase(fromNode.idx);
                            loc_nodes[make_pair(toNode.x, toNode.y)].erase(toNode.idx);
                            fromNode.y = best_y;
                            toNode.y = best_y;
                            loc_nodes[make_pair(fromNode.x, fromNode.y)].insert(fromNode.idx);
                            loc_nodes[make_pair(toNode.x, toNode.y)].insert(toNode.idx);
                            shifted = true;
                            break;
                        } else {  // check next candidate
                            continue;
                        }
                    }
                }
            }
        }
    };

    auto mergeNode = [&]() {
        while (1) {
            bool found_to_merge = false;
            pair<int, int> merge_loc;
            for (auto it = loc_nodes.begin(); it != loc_nodes.end(); it++) {
                if (it->second.size() > 1) {
                    found_to_merge = true;
                    merge_loc = it->first;
                    break;
                }
            }
            if (found_to_merge) {
                set<int>& nodeset = loc_nodes[merge_loc];
                int to_idx = -1;
                int from_idx = -1;
                assert(nodeset.size() == 2);  // only possibility: 1 steiner point + (another steiner point/pin point)
                int two_steiner = -1;

                for (auto idx : nodeset) {
                    assert(idx != -1);
                    if (cur_routeNodes[idx].pinIdxs.size() > 0) {
                        two_steiner = idx;
                    }
                }
                if (two_steiner != -1) {
                    to_idx = two_steiner;
                } else {
                    to_idx = *nodeset.begin();
                }
                for (auto idx : nodeset) {
                    if (idx != to_idx) {
                        from_idx = idx;
                        break;
                    }
                }

                assert(to_idx != -1);
                assert(from_idx != -1);
                nodeset.erase(from_idx);

                auto& fromNode = cur_routeNodes[from_idx];
                auto& toNode = cur_routeNodes[to_idx];
                if (toNode.toConnect.find(from_idx) != toNode.toConnect.end()) {  // two nodes are connected
                    int del_edge = -1;
                    for (int i = 0; i < edgeset.size(); i++) {
                        if (edgeset[i].from == from_idx || edgeset[i].from == to_idx) {
                            if (edgeset[i].to == from_idx || edgeset[i].to == to_idx) {
                                del_edge = i;
                                break;
                            }
                        }
                    }
                    edgeset.erase(edgeset.begin() + del_edge);
                    toNode.toConnect.erase(from_idx);
                }
                for (int j = 0; j < edgeset.size(); j++) {
                    if (edgeset[j].from == from_idx) {
                        edgeset[j].from = to_idx;
                        cur_routeNodes[edgeset[j].to].toConnect.erase(from_idx);
                        cur_routeNodes[edgeset[j].to].toConnect.insert(to_idx);
                        toNode.toConnect.insert(edgeset[j].to);
                    } else if (edgeset[j].to == from_idx) {
                        edgeset[j].to = to_idx;
                        cur_routeNodes[edgeset[j].from].toConnect.erase(from_idx);
                        cur_routeNodes[edgeset[j].from].toConnect.insert(to_idx);
                        toNode.toConnect.insert(edgeset[j].from);
                    }
                }

                cur_routeNodes.erase(from_idx);  // remove node
                // remove redundant edge
                set<int> toDelete;
                for (int i = 0; i < edgeset.size(); i++) {
                    for (int j = i + 1; j < edgeset.size(); j++) {
                        if (edgeset[i].from == edgeset[j].from || edgeset[i].from == edgeset[j].to) {
                            if (edgeset[i].to == edgeset[j].from || edgeset[i].to == edgeset[j].to) toDelete.insert(i);
                        }
                    }
                }
                for (auto idx : toDelete) {
                    edgeset.erase(edgeset.begin() + idx);
                }
            } else {
                break;
            }
        }
    };
    grDatabase.tot_edge += edgeset.size();
    int while_limit = 10000;  // prevent possible(nearly impossible) infinite loop
    int tmp_cnt = 0;
    while (tmp_cnt < while_limit) {
        bool edgeshift = false;
        shiftEdge(edgeshift);
        if (edgeshift == false) break;
        mergeNode();
        tmp_cnt += 1;
    }
    if (tmp_cnt == while_limit) log() << "More Shifting is actually needed" << std::endl;
    grDatabase.edge_shifted += tmp_cnt;

    s = cur_routeNodes.begin()->first;
    queue<int> tmp_q_add;
    tmp_q_add.push(s);
    vis.clear();
    while (tmp_q_add.empty() == false) {
        int u = tmp_q_add.front();
        tmp_q_add.pop();
        vis.insert(u);
        RouteNode& node = cur_routeNodes[u];
        for (int nIdx : node.toConnect) {
            if (vis.find(nIdx) != vis.end()) continue;
            addUsage2D(node, cur_routeNodes[nIdx], 1);  // update the usage
            tmp_q_add.push(nIdx);
        }
    }
}

void InitRoute::getRoutingOrder() {
    // local net in one gcell
    if (routeNodes.size() == 1) return;
    // pick a degree 1 node as root
    int root = -1;
    for (auto it = routeNodes.begin(); it != routeNodes.end(); it++) {
        if (it->second.degree() == 1) {
            root = it->first;
            break;
        }
    }

    // dfs to decide routing order
    set<int> visited;
    function<void(int)> dfs = [&](int nodeIdx) {
        visited.insert(nodeIdx);
        RouteNode& node = routeNodes[nodeIdx];
        for (int nIdx : node.toConnect) {
            if (visited.find(nIdx) != visited.end()) continue;
            RouteEdge edge;
            edge.from = nIdx;
            edge.to = nodeIdx;
            routeEdges.emplace_back(edge);
            routeNodes[edge.to].childIdxs.emplace_back(edge.from);
            dfs(nIdx);
        }
    };
    if (root != -1) {
        dfs(root);
        reverse(routeEdges.begin(), routeEdges.end());
    } else {
        log() << "Error: Can't find a degree one node (2)" << endl;
    }
}

void InitRoute::addUsage2D(RouteNode& u, RouteNode& v, double usage) {
    if (u.x == v.x) {  // straight edge dir = X
        int gridline = u.x;
        utils::IntervalT<int> Range(min(u.y, v.y), max(u.y, v.y));
        for (int cp = Range.low; cp < Range.high; cp++) {
            grDatabase.useWire2D(X, gridline, cp, usage);
        }
    } else if (u.y == v.y) {
        int gridline = u.y;
        utils::IntervalT<int> Range(min(u.x, v.x), max(u.x, v.x));
        for (int cp = Range.low; cp < Range.high; cp++) {
            grDatabase.useWire2D(Y, gridline, cp, usage);
        }
    } else {  // diagnal edge
        RouteNode new1(u.x, v.y), new2(v.x, u.y);
        addUsage2D(new1, u, usage / 2);
        addUsage2D(new1, v, usage / 2);
        addUsage2D(new2, u, usage / 2);
        addUsage2D(new2, v, usage / 2);
    }
}

void InitRoute::removeUsage2D(RouteNode& u, RouteNode& v, double usage) {
    if (u.x == v.x) {  // straight edge dir = Y
        int gridline = u.x;
        utils::IntervalT<int> Range(min(u.y, v.y), max(u.y, v.y));
        for (int cp = Range.low; cp < Range.high; cp++) {
            grDatabase.removeUsage2D(X, gridline, cp, usage);
        }
    } else if (u.y == v.y) {
        int gridline = u.y;
        utils::IntervalT<int> Range(min(u.x, v.x), max(u.x, v.x));
        for (int cp = Range.low; cp < Range.high; cp++) {
            grDatabase.removeUsage2D(Y, gridline, cp, usage);
        }
    } else {  // diagnal edge
        RouteNode new1(u.x, v.y), new2(v.x, u.y);
        removeUsage2D(new1, u, usage / 2);
        removeUsage2D(new1, v, usage / 2);
        removeUsage2D(new2, u, usage / 2);
        removeUsage2D(new2, v, usage / 2);
    }
}

std::map<int, RouteNode>& InitRoute::getRouteNodes() { return routeNodes; }
