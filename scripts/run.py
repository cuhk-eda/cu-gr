#!/usr/bin/env python3

import argparse
import os
import datetime
from run_base import *

# constants
binary = 'iccad19gr'

# argparse
parser = argparse.ArgumentParser()
parser.add_argument('benchmarks', choices=all_benchmarks.get_choices(), nargs='+', metavar='BENCHMARK',
                    help='Choices are ' + ', '.join(all_benchmarks.get_choices()))
parser.add_argument('-m', '--mode', choices=modes)
parser.add_argument(
    '-s', '--steps', choices=['route', 'eval', 'view'], nargs='+', default=['route'])
parser.add_argument('-p', '--benchmark_path')
parser.add_argument('-t', '--threads', type=int, default=8)
args = parser.parse_args()

# seleted benchmarks
bms = all_benchmarks.get_selected(args.benchmarks)
bm_path = args.benchmark_path
if bm_path is None:
    bm_path = os.environ.get('BENCHMARK_PATH')
    if bm_path is None:
        print('Benchmark path is not specified.')
        quit()


# mode cmd_prefix
cmd_prefix = mode_prefixes[args.mode]
if args.mode == 'valgrind':
    print('Please make sure the binary is not compiled with static linking to avoid false alarm')

# run
if True:
    now = datetime.datetime.now()
    log_dir = 'run{:02d}{:02d}'.format(now.month, now.day)

run('mkdir -p {}'.format(log_dir))
print('The following benchmarks will be ran: ', bms)


def route():
    guide_file = '{0}/{1}.solution.guide'.format(bm_log_dir, bm.full_name)
    log_file = '{0}/{1}.log'.format(bm_log_dir, bm.full_name)
    
    run('{cmd_prefix} ./{0} -lef {1}.input.lef -def {1}.input.def -threads {2} -output {3} |& tee {4}'.format(
        binary, file_name_prefix, args.threads, guide_file, log_file, cmd_prefix=cmd_prefix))

    if args.mode == 'gprof':
        run('gprof {} > {}.gprof'.format(binary, bm.full_name))
        run('./gprof2dot.py -s {0}.gprof | dot -Tpdf -o {0}.pdf'.format(bm.full_name))

    run('mv *.solution.guide* *.log *.gprof *.pdf {} 2>/dev/null'.format(bm_log_dir))


def evaluate():
    guide_file = '{0}/{1}.solution.guide'.format(bm_log_dir, bm.full_name)
    sol_file = '{0}/{1}.solution.def'.format(bm_log_dir, bm.full_name)
    dr_log_file = '{0}/{1}_dr_eval.log'.format(bm_log_dir, bm.full_name)
    log_file = '{0}/{1}_eval.log'.format(bm_log_dir, bm.full_name)
    dr_file = 'drcu'
    bm_yy = int(bm.full_name[4:6])
    run('cp ispd{0}eval/ispd{0}eval* ./'.format(bm_yy))

    def evaluate_once():
        run('./{0} -lef {1}.input.lef -def {1}.input.def -guide {2} -threads {3} -tat 2000000000 -output {4} |& tee {5}'.format(
            dr_file, file_name_prefix, guide_file, args.threads, sol_file, dr_log_file))

        if bm_yy == 18:
            run('./ispd{0}eval.sh -lef {1}.input.lef -guide {2} -def {3} -thread {4} |& tee {5}'.format(
                bm_yy, file_name_prefix, guide_file, sol_file, args.threads, log_file))
        else:
            run('./ispd{0}eval.sh -lef {1}.input.lef -guide {2} -idef {1}.input.def -odef {3} -thread {4} |& tee {5}'.format(
                bm_yy, file_name_prefix, guide_file, sol_file, args.threads, log_file))

    evaluate_once()

    run('rm ispd{0}eval_bin ispd{0}eval.sh ispd{0}eval.tcl ispd{0}eval.w *.def.v eval.def'.format(bm_yy))
    run('mv *.log innovus.* *.solution.def* eval.*.rpt {} 2>/dev/null'.format(bm_log_dir))


def view():
    file = open("tmp.tcl", "w")
    file.write("loadLefFile {}.input.lef\n".format(file_name_prefix))
    file.write("loadDefFile {}/{}.solution.def\n".format(bm_log_dir, bm.full_name))
    file.write("setMultiCpuUsage -localCpu {}\n".format(args.threads))
    file.write("win\n")
    file.close()
    run('innovus -init tmp.tcl')
    run('rm innovus.* *.drc.rpt *.solution.def.v tmp.tcl')


for bm in bms:
    bm_log_dir = '{}/{}'.format(log_dir, bm.abbr_name)
    file_name_prefix = '{0}/iccad2019c/{1}/{1}'.format(bm_path, bm.full_name)

    run('mkdir -p {}'.format(bm_log_dir))
    if 'route' in args.steps:
        route()
    if 'eval' in args.steps:
        evaluate()
    if 'view' in args.steps:
        view()
