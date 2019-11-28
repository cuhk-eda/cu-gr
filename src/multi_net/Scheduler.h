#pragma once

#include "single_net/SingleNetRouter.h"

class Scheduler {
public:
    Scheduler(const vector<SingleNetRouter>& routersToExec) : routers(routersToExec){};
    vector<vector<int>>& schedule();

private:
    const vector<SingleNetRouter>& routers;
    vector<vector<int>> batches;

    // for conflict detect
    RTrees rtrees;
    void initSet(vector<int> jobIdxes);
    void updateSet(int jobIdx);
    bool hasConflict(int jobIdx);
};