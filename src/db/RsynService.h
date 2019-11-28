#pragma once

#include "global.h"

namespace db {

class RsynService {
public:
    Rsyn::PhysicalService* physicalService;
    // Rsyn::RoutingGuide* routeGuideService;
    Rsyn::PhysicalDesign physicalDesign;
    Rsyn::Design design;
    Rsyn::Module module;

    void init() {
        Rsyn::Session session;

        physicalService = session.getService("rsyn.physical");
        // routeGuideService = session.getService("rsyn.routingGuide");

        physicalDesign = physicalService->getPhysicalDesign();
        design = session.getDesign();
        module = design.getTopModule();
    }
};

}