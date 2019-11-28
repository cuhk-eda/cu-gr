#pragma once

#include "db/Database.h"
#include "gr_db/GrDatabase.h"
#include "single_net/SingleNetRouter.h"
#include "CongestionMap.h"

class Router {
public:
    Router();
    void run();
private:
    int iter;
    
    vector<db::RouteStatus> allNetStatus;
    CongestionMap congMap;
    int cellWidth = 5, cellHeight = 5;

    vector<int> getNetsToRoute();
    void sortNets(vector<int>& netsToRoute);
    vector<vector<int>> getBatches(vector<SingleNetRouter>& routers, const vector<int>& netsToRoute);

    void routeApprx(const vector<int>& netsToRoute);
    void fluteAllAndRoute(const vector<int>& netsToRoute);

    void ripup(const vector<int>& netsToRoute);
    void updateCost();

    void printStat();
};
