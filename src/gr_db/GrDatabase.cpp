#include "GrDatabase.h"
#include <fstream>

gr::GrDatabase grDatabase;

namespace gr {
void GrDatabase::init() {
    GrRouteGrid::init();
    GrNetlist::init(*this);

    GrRouteGrid::print();
}

void GrDatabase::writeGuides(std::string filename) {
    log() << "Writing guides to file..." << std::endl;

    std::stringstream ss;

    auto printGrGuides = [&](const vector<GrBoxOnLayer>& guides) {
        for (const auto& guide : guides) {
            ss << getCoor(guide[X].low, X) << " ";
            ss << getCoor(guide[Y].low, Y) << " ";
            ss << getCoor(guide[X].high + 1, X) << " ";
            ss << getCoor(guide[Y].high + 1, Y) << " ";
            ss << database.getLayer(guide.layerIdx).name << std::endl;
        }
    };

    for (const auto& net : grDatabase.nets) {
        ss << net.getName() << std::endl;
        ss << "(" << std::endl;
        printGrGuides(net.wireRouteGuides);
        printGrGuides(net.viaRouteGuides);
        printGrGuides(net.patchRouteGuides);
        ss << ")" << std::endl;
    }

    std::ofstream fout(filename);
    fout << ss.str();
    fout.close();
}
}  // namespace gr
