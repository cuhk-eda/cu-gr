#!/bin/sh

if [ $# -eq 0 ] ; then
    echo "$0 -lef <input LEF> -guide <guide file> -idef <input DEF> -odef <routed DEF>"
    echo "example:"
    echo "$0 -lef sample/ispd19_sample.input.lef -guide sample/ispd19_sample.input.guide -idef sample/ispd19_sample.input.def -odef sample/ispd19_sample.output.def"
    exit
fi

INVS="innovus"
EVAL="./ispd19eval_bin"

command -v $INVS || {
    echo "$INVS is not available."
    exit 1
}

method='a'
keepLogs=0
prefix=
trimmedDef='eval.def'
combinedDef='comb.def'

#read arguments
while [ $# -gt 0 ] ; do
    case $1 in
        -lef)   shift ; inputLef=$1 ;;
        -guide) shift ; inputRg=$1 ;;
        -idef)  shift ; inputDef=$1 ;;
        -odef)  shift ; outputDef=$1 ;;
        -tdef)  shift ; trimmedDef=$1 ;;
        -cdef)  shift ; combinedDef=$1 ;;
        -prefix) shift ; prefix=$1 ;;
        -method) shift ; method=$1 ;;
        -keep) keepLogs=1 ;;
        *) echo "unknown : $1" ;;
    esac
    shift
done

evalGeoRpt="${prefix}eval.geo.rpt"
evalConRpt="${prefix}eval.con.rpt"
evalScoreRpt="${prefix}eval.score.rpt"
evalTcl="${prefix}eval.tcl"
invsLog="${prefix}eval"

if [ ! -f $inputLef ] ; then
    echo "ERROR: $inputLef not found"
    exit
elif [ ! -f $inputRg ] ; then
    echo "ERROR: $inputRg not found"
    exit
elif [ ! -f $inputDef ] ; then
    echo "ERROR: $inputDef not found"
    exit
elif [ ! -f $outputDef ] ; then
    echo "ERROR: $outputDef not found"
    exit
fi

if [ -f $evalTcl ] ; then
    echo "WARNING: tcl file exists, overwriting..."
fi

if [ "$method" = 'a' ] ; then
    dbuMicron=`grep "UNITS DISTANCE MICRONS" $inputDef | cut -d' ' -f 4`
    echo 'VERSION 5.8 ;' > $trimmedDef
    echo 'DIVIDERCHAR "/" ;' >> $trimmedDef
    echo 'BUSBITCHARS "[]" ;' >> $trimmedDef
    echo "UNITS DISTANCE MICRONS $dbuMicron ;" >> $trimmedDef
    sed -n '/^[[:blank:]]*NETS[[:blank:]]*[1-9][0-9]*[[:blank:]]*;[[:blank:]]*$/,/^[[:blank:]]*END NETS[[:blank:]]*$/p' $outputDef >> $trimmedDef
elif [ "$method" = 'b' ] ; then
    sed -n '/^[[:blank:]]*VERSION[[:blank:]]*[0-9]\.[0-9][[:blank:]]*;[[:blank:]]*$/,/^END SPECIALNETS[[:blank:]]*$/p' $inputDef > $combinedDef
    sed -n '/^[[:blank:]]*NETS[[:blank:]]*[1-9][0-9]*[[:blank:]]*;[[:blank:]]*$/,/^[[:blank:]]*END NETS[[:blank:]]*$/p' $outputDef >> $combinedDef
    echo "END DESIGN" >> $combinedDef

else
    echo "ERROR: unknown evaluation method $method"
    exit
fi

echo "#evaluation script for ISPD 2019 contest" > $evalTcl
echo 'source ispd19eval.tcl' >> $evalTcl
if [ "$method" = 'a' ] ; then
    echo "evaluate $inputLef $inputDef $trimmedDef $evalGeoRpt $evalConRpt" >> $evalTcl
else
    echo "evaluate $inputLef $combinedDef {} $evalGeoRpt $evalConRpt" >> $evalTcl
fi

echo 'exit' >>$evalTcl

if [ "$keepLogs" = 0 ] ; then
    rm -f $evalGeoRpt
    rm -f $evalConRpt
fi

cmd="$INVS -init $evalTcl -log $invsLog.log -overwrite -nowin"
echo $cmd
$cmd

if [ "$keepLogs" = 0 ] ; then
    rm -f $evalTcl
    rm -f "$invsLog.log"
    rm -f "$invsLog.logv"
    rm -f "$invsLog.cmd"
    rm -f `basename "$outputDef.v"`
fi

cmd="$EVAL -lef $inputLef -def $outputDef -guide $inputRg -georpt $evalGeoRpt -conrpt $evalConRpt"
echo $cmd
$cmd | tee $evalScoreRpt

