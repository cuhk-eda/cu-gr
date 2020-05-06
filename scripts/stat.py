#!/usr/bin/env python3

import argparse, os
from run_base import *
from stat_base import *

parser = argparse.ArgumentParser()
parser.add_argument('--dir', '-d')
parser.add_argument('--files', '-f', type=str, nargs='+')
args = parser.parse_args()

route_items = ['wire space', 'wire-wire', 'wire-fix', 'via-via', 'via-wire', 'via-fix', 'min area', 'est score', 'mem', 'runtime']
eval_items = ['WL', '#sv', 'og WL', 'og #v', 'ot WL', 'ot #v', 'ww WL', '#short', 'short a', '#min a', '#prl', '#eol', '#cut', '#adj', '#cnr', '#spc', '#o', 'total score', 'GR time (sec)', 'DR time (sec)']

def ParseRoute(file):
    # register
    reRoute = RegexPatternSet()
    # reRoute.Add(['# fails'], '.* ([\d.]+) fail.', False, 'Finish all RRR iterations and PostRoute')  # TODO: clearly print out in log
    reRoute.Add(['wire space', 'wire-wire', 'wire-fix'], '.*SumW[ ]+\| ([\d.]+)[ ]+\| [\d.]+[ ]+\| [\d.]+[ ]+\| ([\d.]+)[ ]+\| ([\d.]+)[ ]+', False, 'Finish all RRR iterations and PostRoute')
    reRoute.Add(['via-via', 'via-wire', 'via-fix'], '.*BigSumV[ ]+\| ([\d.]+)[ ]+\| ([\d.]+)[ ]+\| ([\d.]+)[ ]+', False, 'Finish all RRR iterations and PostRoute')
    reRoute.Add(['min area'], '.*# min-area vio: ([\d.]+)', False, 'Finish all RRR iterations and PostRoute')
    reRoute.Add(['est score'], '.*total score = ([\d.e+]+)', False, 'Finish all RRR iterations and PostRoute')
    reRoute.Add(['mem'], '\tMaximum resident set size \(kbytes\): ([\d]+)')
    reRoute.Add(['runtime'], '\tElapsed \(wall clock\) time \(h:mm:ss or m:ss\): ([:\d.]+)')

    # iterate lines
    result = {}
    with open(file) as f:
        for line in f:
            reRoute.ProcessLine(result, line)
    if 'mem' in result:
        result['mem'] = str(round(float(result['mem']) / 1048576, 6)) # KB to GB
    return result

def AppendRuntime(file, item_name, result):
    # register
    print(file)
    reRoute = RegexPatternSet()
    # reRoute.Add(['runtime'], '\tElapsed \(wall clock\) time \(h:mm:ss or m:ss\): ([:\d.]+)')
    reRoute.Add([item_name], '\[ *([:\d.]+)\][ ]+Terminated...')

    # iterate lines
    with open(file) as f:
        for line in f:
            reRoute.ProcessLine(result, line)
    # if 'mem' in result:
    #     result['mem'] = str(round(float(result['mem']) / 1048576, 6)) # KB to GB
    print(result)
    return result

def ParseEval(file):
    # register
    reRoute = RegexPatternSet()
    reRoute.Add(['WL'], '.*Total wire length \|[ ]+[\d.]+ \|[ ]+([\d.]+)')
    reRoute.Add(['#sv'], '.*Total via count \|[ ]+([\d.]+)')
    reRoute.Add(['#sv'], '.*Total SCut via count \|[ ]+([\d.]+)')
    # reRoute.Add(['#mv'], '.*Total MCut via count \|[ ]+([\d.]+)')
    reRoute.Add(['og WL'], '.*Out-of-guide wire \|[ ]+[\d.]+ \|[ ]+([\d.]+)')
    reRoute.Add(['og #v'], '.*Out-of-guide vias \|[ ]+([\d.]+)')
    reRoute.Add(['ot WL'], '.*Off-track wire \|[ ]+[\d.]+ \|[ ]+([\d.]+)')
    reRoute.Add(['ot #v'], '.*Off-track via \|[ ]+([\d.]+)')
    reRoute.Add(['ww WL'], '.*Wrong-way wire \|[ ]+[\d.]+ \|[ ]+([\d.]+)')
    reRoute.Add(['#short'], '.*Number of metal/cut shorts \|[ ]+([\d.]+)')
    reRoute.Add(['short a'], '.*Area of metal shorts \|[ ]+[\d.]+ \|[ ]+([\d.]+)')
    reRoute.Add(['short a'], '.*Area of metal/cut shorts \|[ ]+[\d.]+ \|[ ]+([\d.]+)')
    reRoute.Add(['#min a'], '.*#min-area violations \|[ ]+([\d.]+)')
    reRoute.Add(['#prl'], '.*#PRL violation \|[ ]+([\d.]+)')
    reRoute.Add(['#eol'], '.*#EOL Spacing violation \|[ ]+([\d.]+)')
    reRoute.Add(['#cut'], '.*#Cut Spacing violation \|[ ]+([\d.]+)')
    reRoute.Add(['#adj'], '.*#Adj. Cut Spacing violation \|[ ]+([\d.]+)')
    reRoute.Add(['#cnr'], '.*#Corner Spacing violation \|[ ]+([\d.]+)')
    reRoute.Add(['#spc'], '.*#spacing violations \|[ ]+([\d.]+)')
    reRoute.Add(['#o'], '.*#open nets \|[ ]+([\d.]+)')
    reRoute.Add(['total score'], '.*Total Score \|[ ]+([\d.]+)')

    # iterate lines
    result = {}
    with open(file) as f:
        for line in f:
            reRoute.ProcessLine(result, line)
    return result

def IsValidFile(file):
    if not os.path.isfile(file):
        print('Warning: file', file, 'does not exist')
        return False
    print('Processing file', file, '...')
    return True

def IsEvalRpt(file):
    with open(file) as f:
        for line in f:
            if 'Guides and tracks Obedience' in line:
                return True
    return False

route_results = []
route_time_results = []
eval_results = []

if args.files:
    for file in args.files:
        if not IsValidFile(file):
            continue
        if IsEvalRpt(file):
            eval_results.append(ParseEval(file))
        else:
            route_results.append(ParseRoute(file))
elif args.dir:
    # route_items = ['case'] + route_items
    eval_items = ['design'] + eval_items
    for bm in all_benchmarks.get_selected('all'):
        gr_file = '{}/{}/{}.log'.format(args.dir, bm.abbr_name, bm.full_name)
        dr_file = '{}/{}/{}_dr_eval.log'.format(args.dir, bm.abbr_name, bm.full_name)
        eval_file = '{}/{}/eval.score.rpt'.format(args.dir, bm.abbr_name)
        # if IsValidFile(route_file):
        #     res = ParseRoute(route_file)
        #     res = AppendRuntime(route_file, 'GR time (sec)', res)
        #     route_results.append(res)
        #     route_results[-1]['case'] = bm.abbr_name
        if IsValidFile(eval_file):
            res = ParseEval(eval_file)
            AppendRuntime(gr_file, 'GR time (sec)', res)
            AppendRuntime(dr_file, 'DR time (sec)', res)
            eval_results.append(res)
            eval_results[-1]['design'] = '`' + bm.full_name + '`'
else:
    print('Error: neither file(s) nor directory specified')
    quit()

# print('### ROUTER INFO')
# PrintResultTable(route_items, route_results)
# print('')
print('### EVALUATOR INFO')
PrintResultTable(eval_items, eval_results)
