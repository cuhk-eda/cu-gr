#!/bin/sh

if [ $# -eq 0 ] ; then
    echo "$0 -lef <input LEF> -guide <guide file> -def <routed DEF>"
    echo "example:"
    echo "$0 -lef sample/ispd18_sample.input.lef -guide sample/ispd18_sample.input.guide -def sample/ispd18_sample.output.def"
    exit
fi

INVS="innovus"
EVAL="./ispd18eval_bin"

command -v $INVS || {
    echo "$INVS is not available."
    exit 1
}

keepLogs=0

#read arguments
while [ $# -gt 0 ] ; do
    case $1 in
        -lef)   shift ; inputLef=$1 ;;
        -guide) shift ; inputRg=$1 ;;
        -def)   shift ; outputDef=$1 ;;
        -keep)  shift ; keepLogs=1 ;;
        *) echo "unknown : $1" ;;
    esac
    shift
done

evalGeoRpt='eval.geo.rpt'
evalConRpt='eval.con.rpt'
evalScoreRpt='eval.score.rpt'
evalTcl='eval.tcl'

if [ ! -f $inputLef ] ; then
    echo "ERROR: $inputLef not found"
    exit
elif [ ! -f $inputRg ] ; then
    echo "ERROR: $inputRg not found"
    exit
elif [ ! -f $outputDef ] ; then
    echo "ERROR: $outputDef not found"
    exit
fi

if [ -f $evalTcl ] ; then
    echo "WARNING: tcl file exists, overwriting..."
fi

echo "#evaluation script for ISPD 2018 contest" > $evalTcl
echo 'source ispd18eval.tcl' >> $evalTcl
echo "evaluate $inputLef $outputDef $evalGeoRpt $evalConRpt" >> $evalTcl
echo 'exit' >>$evalTcl

if [ "$keepLogs" = 0 ] ; then
    rm -f $evalGeoRpt
    rm -f $evalConRpt
fi

cmd="$INVS -init $evalTcl -log eval.log -overwrite -nowin"
echo $cmd
$cmd

if [ "$keepLogs" = 0 ] ; then
    rm $evalTcl
    rm 'eval.log'
    rm 'eval.logv'
    rm 'eval.cmd'
    rm `basename "$outputDef.v"`
fi

cmd="$EVAL -lef $inputLef -def $outputDef -guide $inputRg -georpt $evalGeoRpt -conrpt $evalConRpt"
echo $cmd
$cmd | tee $evalScoreRpt

