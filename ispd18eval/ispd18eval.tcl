
proc evaluate { inputLef outputDef evalGeoRpt evalConRpt } {
    setMultiCpuUsage -localCpu 8
    loadDesign $inputLef $outputDef

    setVerifyGeometryMode -area { 0 0 0 0 }
    setVerifyGeometryMode -minWidth false
    setVerifyGeometryMode -minSpacing true
    setVerifyGeometryMode -minArea true
    setVerifyGeometryMode -sameNet true
    setVerifyGeometryMode -short true
    setVerifyGeometryMode -overlap false
    setVerifyGeometryMode -offRGrid false
    setVerifyGeometryMode -offMGrid false
    setVerifyGeometryMode -mergedMGridCheck false
    setVerifyGeometryMode -minHole false
    setVerifyGeometryMode -implantCheck false
    setVerifyGeometryMode -minimumCut false
    setVerifyGeometryMode -minStep false
    setVerifyGeometryMode -viaEnclosure false
    setVerifyGeometryMode -antenna false
    setVerifyGeometryMode -insuffMetalOverlap false
    setVerifyGeometryMode -pinInBlkg false
    setVerifyGeometryMode -diffCellViol false
    setVerifyGeometryMode -sameCellViol false
    setVerifyGeometryMode -padFillerCellsOverlap false
    setVerifyGeometryMode -routingBlkgPinOverlap false
    setVerifyGeometryMode -routingCellBlkgOverlap false
    setVerifyGeometryMode -regRoutingOnly false
    setVerifyGeometryMode -stackedViasOnRegNet false
    setVerifyGeometryMode -wireExt false
    setVerifyGeometryMode -useNonDefaultSpacing false
    setVerifyGeometryMode -maxWidth false
    setVerifyGeometryMode -maxNonPrefLength -1
    setVerifyGeometryMode -error 10000000
    catch { verifyGeometry -report $evalGeoRpt }

    catch { verifyConnectivity -noFloatingMetal -error 10000000 -report $evalConRpt }
    #catch { verifyConnectivity -type all -noAntenna -noUnConnPin -geomConnect -noSoftPGConnect -noFloatingMetal -error 10000000 -report $evalConRpt }

    exit
}

proc loadDesign { lef def } {
    catch { loadLefFile $lef }
    catch { loadDefFile $def }
}

