#include "Stat.h"
#include "Setting.h"

namespace db {

RouteStat routeStat;

RouteStatus operator&(const RouteStatus& lhs, const RouteStatus& rhs) {
    for (auto status : {lhs, rhs}) {
        if (!isSucc(status)) {
            return status;
        }
    }
    for (auto status : {lhs, rhs}) {
        if (status != +RouteStatus::SUCC_NORMAL) {
            return status;
        }
    }
    return RouteStatus::SUCC_NORMAL;
}

RouteStatus& operator&=(RouteStatus& lhs, const RouteStatus& rhs) {
    lhs = lhs & rhs;
    return lhs;
}

std::unordered_map<int, std::string> descriptions = {
    {(+db::RouteStatus::FAIL_PIN_OUT_OF_GRID)._to_integral(), "has pin not on grid, skip"},
    {(+db::RouteStatus::FAIL_DETACHED_GUIDE)._to_integral(), "has route guide detached from others, skip"},
    {(+db::RouteStatus::FAIL_DETACHED_PIN)._to_integral(),
     "has pin detached from route guides, cannot fix by expanding route guides"},
    {(+db::RouteStatus::FAIL_CONN_EXT_PIN)._to_integral(),
     "has unconnected pin due to access box inflation, cannot fix by add linking wire"},
    {(+db::RouteStatus::FAIL_DISCONNECTED_GRID_GRAPH)._to_integral(), "has disconnected grid graph, skip"}};

std::mutex printWarnMsgMutex;

void printWarnMsg(RouteStatus status, const Net& net) {
    if (setting.dbVerbose < +db::VerboseLevelT::MIDDLE || isSucc(status)) {
        return;
    }
    printWarnMsgMutex.lock();
    static std::unordered_map<int, int> counts;
    int& count = counts[status._to_integral()];
    if (count >= db::setting.maxNumWarnForEachRouteStatus) {
        printWarnMsgMutex.unlock();
        return;
    }
    std::string desc;
    auto it = descriptions.find(status._to_integral());
    if (it != descriptions.end()) {
        desc = " (" + it->second + ")";
    }
    log() << "Warning: Net " << net.getName() << " gets " << status._to_string() << desc << std::endl;
    ++count;
    if (count == db::setting.maxNumWarnForEachRouteStatus) {
        log() << "More warnings on " << status._to_string() << " will be suppressed" << std::endl;
        log() << std::endl;
    }
    printWarnMsgMutex.unlock();
}

void StageRouteStat::clear() {
    allNetStatusCounters.clear();
    miscEventCounters.clear();
}

std::mutex stageRouteStatusMutex;
std::mutex miscEventMutex;

void StageRouteStat::increment(RouteStatus status) {
    stageRouteStatusMutex.lock();
    ++allNetStatusCounters[status._to_integral()];
    stageRouteStatusMutex.unlock();
}

void StageRouteStat::increment(MiscRouteEvent misc, int count) {
    miscEventMutex.lock();
    miscEventCounters[misc._to_integral()] += count;
    miscEventMutex.unlock();
}

void StageRouteStat::print(const std::string& stageStr) const {
    // allNetStatusCounters
    int numTotal = 0, numSucc = 0;
    iterateEnumCountersInOrder<RouteStatus, int>(allNetStatusCounters, [&](RouteStatus status, int count) {
        numTotal += count;
        if (isSucc(status)) {
            numSucc += count;
        }
    });
    int numFail = numTotal - numSucc;
    log() << stageStr << ": #nets = " << numTotal << std::endl;
    log() << "\t#succ = " << numSucc << " (";
    int i = 0;
    iterateEnumCountersInOrder<RouteStatus, int>(allNetStatusCounters, [&](RouteStatus status, int count) {
        if (isSucc(status)) {
            std::cout << "#" << status << " = " << count << " ";
        }
        ++i;
    });
    std::cout << ")" << std::endl;
    if (numFail > 0) {
        log() << "\t#fail = " << numFail << " (";
        i = 0;
        iterateEnumCountersInOrder<RouteStatus, int>(allNetStatusCounters, [&](RouteStatus status, int count) {
            if (!isSucc(status)) {
                std::cout << "#" << status << " = " << count << " ";
            }
            ++i;
        });
        std::cout << ")" << std::endl;
    }

    // miscEventCounters
    if (!miscEventCounters.empty()) {
        log() << "\tmisc (";
        iterateEnumCountersInOrder<MiscRouteEvent, int>(miscEventCounters, [&](MiscRouteEvent event, int count) {
            std::cout << "#" << event << " = " << count << " ";
        });
        std::cout << ")" << std::endl;
    }
}

void RouteStat::clear() { stages.clear(); }

void RouteStat::print() const {
    iterateEnumCountersInOrder<RouteStage, StageRouteStat>(stages, [](RouteStage stage, StageRouteStat stageStat) {
        stageStat.print("Stage " + std::string(stage._to_string()));
    });
}

}  // namespace db
