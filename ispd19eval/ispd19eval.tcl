setMultiCpuUsage -localCpu 8
proc evaluate { inputLef inputDef outputDef evalGeoRpt evalConRpt } {

    loadDesign $inputLef $inputDef $outputDef
    
    set_verify_drc_mode -disable_rules {}
    set_verify_drc_mode -check_implant true
    set_verify_drc_mode -check_implant_across_rows false
    set_verify_drc_mode -check_ndr_spacing false
    set_verify_drc_mode -check_only default
    set_verify_drc_mode -check_same_via_cell false
    set_verify_drc_mode -exclude_pg_net false
    set_verify_drc_mode -ignore_trial_route false
    set_verify_drc_mode -report $evalGeoRpt
    set_verify_drc_mode -limit 10000000
    catch { verify_drc }

#   catch { verifyConnectivity -noFloatingMetal -error 10000000 -report $evalConRpt }
    catch { verifyConnectivity -type all -noAntenna -noUnConnPin -geomConnect -noSoftPGConnect -noFloatingMetal -error 10000000 -report $evalConRpt }

    exit
}

proc loadDesign { lef idef odef } {
    catch { loadLefFile $lef }
    catch { loadDefFile $idef }
    if { $odef != "" } {
        catch { defIn $odef }
    }
}

