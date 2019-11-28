#pragma once

#include "db/Database.h"
#include "GrRouteGrid.h"
#include "global.h"
#include "GrNet.h"

namespace gr {
class GrDatabase : public GrRouteGrid, public GrRouteGrid2D, public GrNetlist {
public:
    void init();
    void writeGuides(std::string filename);

private:
};

}  // namespace gr

extern gr::GrDatabase grDatabase;