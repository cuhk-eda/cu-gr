#pragma once

#include "Net.h"
#include "global.h"

namespace db {

BETTER_ENUM(RouteStatus,
            int,
            // 1. Succ
            SUCC_NORMAL,
            // 1.1 pre-route
            SUCC_ONE_PIN,
            // 1.2 post-route
            SUCC_CONN_EXT_PIN,
            // 2. Fail
            // 2.0
            FAIL_UNPROCESSED,
            // 2.1 pre route
            FAIL_PIN_OUT_OF_GRID,
            FAIL_DETACHED_GUIDE,
            FAIL_DETACHED_PIN,
            // 2.2 maze route
            FAIL_DISCONNECTED_GRID_GRAPH,
            // 2.3 post route
            FAIL_CONN_EXT_PIN);

BETTER_ENUM(MiscRouteEvent,
            int,
            // pre route
            ADD_DIFF_LAYER_GUIDE_1,
            ADD_DIFF_LAYER_GUIDE_2,
            FIX_DETACHED_PIN,
            // post maze route
            MIN_AREA_VIO,
            MIN_AREA_SHADOWED_VIO,
            REMOVE_TRACK_SWITCH_PIN,
            REMOVE_TRACK_SWITCH_NORMAL,
            REMOVE_TRACK_SWITCH_HORSESHOE,
            // post route
            LINK_PIN_VIO,
            FILL_SAME_NET_SPACE,
            REMOVE_CORNER);

BETTER_ENUM(RouteStage, int, INIT, MAZE, ALL);

template <typename EnumT, typename MappedT>
void iterateEnumCountersInOrder(const std::unordered_map<int, MappedT>& counters,
                                const std::function<void(EnumT, MappedT)>& handle) {
    for (EnumT enumType : EnumT::_values()) {
        auto it = counters.find(enumType._to_integral());
        if (it != counters.end()) {
            handle(EnumT::_from_integral(it->first), it->second);
        }
    }
}

// switch syntax is not supported by gcc 5 ...
constexpr bool isSucc(RouteStatus status) {
    return status == +RouteStatus::SUCC_NORMAL || status == +RouteStatus::SUCC_ONE_PIN ||
           status == +RouteStatus::SUCC_CONN_EXT_PIN;
}

// constexpr auto succ = better_enums::make_map(isSucc);  // more efficient if many

RouteStatus operator&(const RouteStatus& lhs, const RouteStatus& rhs);
RouteStatus& operator&=(RouteStatus& lhs, const RouteStatus& rhs);

void printWarnMsg(RouteStatus status, const Net& net);

class StageRouteStat {
public:
    void clear();
    void increment(RouteStatus status);
    void increment(MiscRouteEvent misc, int count);
    void print(const std::string& stageStr) const;

private:
    std::unordered_map<int, int> allNetStatusCounters;
    std::unordered_map<int, int> miscEventCounters;
};

class RouteStat {
public:
    template <typename... Args>
    void increment(RouteStage stage, Args... params) {
        routeStatusMutex.lock();
        stages[stage._to_integral()].increment(params...);
        routeStatusMutex.unlock();
    }
    void clear();
    void print() const;

private:
    std::unordered_map<int, StageRouteStat> stages;
    std::mutex routeStatusMutex;
};

extern RouteStat routeStat;

}  // namespace db
