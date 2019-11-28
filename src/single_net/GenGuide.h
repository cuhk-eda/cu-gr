#pragma once
#include "gr_db/GrDatabase.h"

class GuideGenerator {
public:
    GuideGenerator(gr::GrNet& grNetData) : grNet(grNetData) {}
    void genTopoGuides();
    void genPatchGuides();
    static void sliceGuides(vector<gr::GrBoxOnLayer>& guides, bool mergeAdj = false);

private:
    gr::GrNet& grNet;
    void genConnGuides();

    void patchPinRegions();
    void patchLongSegments();
    void patchVioCells();
};

class GuideGeneratorStat {
public:
    int longSegmentPatchNum;

    int vioCellNum;
    int vioCellPatchNum;

    int pinRegionPatchCand;
    int pinRegionPatchNum;

    void reset() {
        longSegmentPatchNum = 0;

        vioCellNum = 0;
        vioCellPatchNum = 0;
    }

    void print() {
        log() << "Add pin region patches. Totally " << pinRegionPatchNum << "/" << pinRegionPatchCand
              << " patches are added." << std::endl;
        log() << "Add long segment patches. Totally " << longSegmentPatchNum << " patches are added." << std::endl;
        printflog("Add cong cell patches. Totally %d/%d patches are added\n", vioCellPatchNum, vioCellNum);
    }
};

extern GuideGeneratorStat guideGenStat;
